/*
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/

// Updates the latest latency and bandwidth
function UpdateLatencyAndBandwith(latency, bandwidth) {
  let view = GetStatisticsView();
  const time = new Date().getTime();
  view.plot_latency_series.append(time, latency, false, true);
  view.plot_bandwith_series.append(time, bandwidth, false, true);
  view.stats_div.removeChild(view.stats_div.lastChild);
  view.stats_div.innerHTML = "Latency: &nbsp; &nbsp; &nbsp;" + latency + "ms<br />";
  if (bandwidth > 1024 * 1024) {
    bandwidth /= (1024 * 1024);
    view.stats_div.innerHTML += "Bandwidth: " + ((100*bandwidth)|0)/100.0 + "mb";
  } else if (bandwidth > 1024) {
    bandwidth /= (1024);
    view.stats_div.innerHTML += "Bandwidth: " + ((10*bandwidth)|0)/10.0 + "kb";
  } else {
    view.stats_div.innerHTML += "Bandwidth: " + ((10*bandwidth)|0)/10.0 + "b";
  }
}

// Update the statistics widget with the latest information coming from the backend.
function UpdateStatistics(data) {
  // The first message can be empty, let's skip it.
  if (!data) return;
  let view = GetStatisticsView();
  // If the view is not visible, we can delay any changes:
  if (!view.isVisible()) {
    if (!view.data) {
      // Let's try again in 1s
      setTimeout(function() {
        if (view.data) {
          UpdateStatistics(view.data);
        }
      }, 1000);
    }
    view.data = data;
    return;
  }
  view.data = null;
  let new_codelet = false;
  for (const node_name in data.codelet_statistics) {
    if (!view.node_codelets[node_name]) {
      view.node_codelets[node_name] = {};
    }
    const node = data.codelet_statistics[node_name];
    for (const codelet_name in node) {
      try {
        let stats = node[codelet_name];
        if (!stats) continue;
        if (!view.node_codelets[node_name][codelet_name]) {
          // If this is a new codelet, we create a new row.
          let row = document.createElement("tr");
          row.classList.add("tabcontent");
          for (let i = 0; i < view.titles.length; i++) {
            let td = document.createElement("td");
            td.classList.add("tabcol"+i);
            row.appendChild(td);
            view.table_div.appendChild(row);
          }
          row.children[0].original_name = node_name;
          row.children[1].original_name = codelet_name;
          view.node_codelets[node_name][codelet_name] = row;
          new_codelet = true;
        }
        let row = view.node_codelets[node_name][codelet_name];
        row.children[2].innerHTML = stats.num_ticks;
        row.children[3].innerHTML = stats.average_exec_dt;
        row.children[4].innerHTML = stats.median;
        row.children[5].innerHTML = stats.p90;
        row.children[6].innerHTML = stats.max;
        row.children[7].innerHTML = stats.frequency;
        row.children[8].innerHTML = stats.load;
        row.children[9].innerHTML = stats.late_p;
      } catch (e) {
        console.error(e);
      }
    }
  }
  if (new_codelet) view.showCodelet();
  view.sort();
}

// Returns the div containing the graph view, if it does not exist yet create it.
function GetStatisticsView() {
  let stats_view = document.getElementById("__win-statistics-view");
  if (stats_view === null || stats_view === undefined) {
    // Create the new widget
    stats_view = document.createElement("div");
    stats_view.style.display = "block";
    stats_view.id = "__win-statistics-view";
    // Create the search input
    let top_bar = document.createElement("div");
    stats_view.appendChild(top_bar);
    top_bar.classList.add("tab");
    top_bar.style.display = "inline-flex";
    let search_div = document.createElement("div");
    search_div.style.width = "calc(100% - 240px)";
    search_div.style.minWidth = "215px";
    top_bar.appendChild(search_div);
    stats_view.input_elem = document.createElement("input");
    stats_view.input_elem.type = "text";
    stats_view.input_elem.placeholder = "Filter";
    search_div.appendChild(stats_view.input_elem);
    stats_view.stats_div = document.createElement("div");
    stats_view.stats_div.style.width = "140px";
    stats_view.stats_div.style.whiteSpace = "nowrap";
    stats_view.stats_div.innerHTML = "Latency: &nbsp; &nbsp; &nbsp;-----ms<br />";
    stats_view.stats_div.innerHTML += "Bandwidth: -----kb";
    top_bar.appendChild(stats_view.stats_div);
    let canvas_div = document.createElement("div");
    canvas_div.style.width = "100px";
    canvas_div.style.height = "40px";
    top_bar.appendChild(canvas_div);
    let canvas_latency = document.createElement("canvas");
    canvas_latency.width = 100;
    canvas_latency.height = 20;

    let smoothie_config = {
      yMinFormatter: function(min, precision) { return "";},
      yMaxFormatter: function(max, precision) { return "";},
      yIntermediateFormatter: function(val, precision) { return "";},
      yLabelFormatter: function(val, precision) { return (parseFloat(val)|0)+"ms";},
      millisPerPixel: 200
    };

    canvas_div.appendChild(canvas_latency);
    stats_view.plot_latency = new SmoothieChart(smoothie_config);
    stats_view.plot_latency_series = new TimeSeries();
    stats_view.plot_latency.streamTo(canvas_latency);
    stats_view.plot_latency.addTimeSeries(stats_view.plot_latency_series,
                                          {lineWidth: 2, strokeStyle: "#f00"});
    let canvas_bandwith = document.createElement("canvas");
    canvas_bandwith.width = 100;
    canvas_bandwith.height = 20;
    canvas_bandwith.style.marginTop = "-4px";
    canvas_div.appendChild(canvas_bandwith);
    smoothie_config.yLabelFormatter = function(val, precision) {
      if (val > 1024*1024) {
        return (((100.0 * val / (1024*1024))|0) / 100.0) + "mb";
      }
      if (val > 1024) {
        return (((10.0 * val / (1024))|0) / 10.0) + "kb";
      }
      return (val|0) + "b";
    };
    stats_view.plot_bandwith = new SmoothieChart(smoothie_config);
    stats_view.plot_bandwith.streamTo(canvas_bandwith);
    stats_view.plot_bandwith_series = new TimeSeries();
    stats_view.plot_bandwith.addTimeSeries(stats_view.plot_bandwith_series,
                                          {lineWidth: 2, strokeStyle: "#0f0"});

    // Parameters of the widget
    stats_view.node_codelets = {};  // Dictionary to easily access the row of a given codelet
    stats_view.sort_col = 0;        // Which column to use to sort
    stats_view.sort_dir = 1;        // Direction of the sort.
    stats_view.filter_str = "";     // Filter used to hide row

    // To enhance visualization, we change the background of every other (visible) row.
    stats_view.colorRows = function() {
      let counter = 0;
      for (let i = 0; i < stats_view.table_div.children.length; i++) {
        if (counter%2) {
          stats_view.table_div.children[i].style.backgroundColor = "#f2f2f2";
        } else {
          stats_view.table_div.children[i].style.backgroundColor = "";
        }
        if (stats_view.table_div.children[i].style.display !== "none") counter++;
      }
    };

    // Sort each row.
    stats_view.sort = function() {
      const sort = function(a, b) {
        // Make sure we don't move the first row
        if (a == stats_view.table_div.childNodes[0]) return -1;
        if (b == stats_view.table_div.childNodes[0]) return 1;
        if (stats_view.sort_col < 2) {
          // Sort lexicographically by default
          const al = a.children[stats_view.sort_col].innerHTML.toLowerCase();
          const bl = b.children[stats_view.sort_col].innerHTML.toLowerCase();
          if (al == bl) return 0;
          return al < bl ? -stats_view.sort_dir : stats_view.sort_dir;
        } else {
          // Sort ascending order by default
          return (parseFloat(a.children[stats_view.sort_col].innerHTML) -
                  parseFloat(b.children[stats_view.sort_col].innerHTML)) * stats_view.sort_dir;
        }
      };
      let change = false;
      const nodes = stats_view.table_div.childNodes;
      for (let i = 1; i < nodes.length; i++) {
        let j = i;
        while (sort(nodes[j - 1], nodes[i]) > 0) j--;
        if (j != i) {
          stats_view.table_div.insertBefore(nodes[i], nodes[j]);
          change = true;
        }
      }
      if (change) stats_view.colorRows();
    };

    // Only display the codelet of a selected node.
    stats_view.showCodelet = function(evt) {
      // If escape is pressed, erase the input
      if (evt && evt.keyCode == 27) {
        stats_view.input_elem.value = "";
      }
      stats_view.filter_str = stats_view.input_elem.value;
      let tabcontent = document.getElementsByClassName("tabcontent");
      let bmatch = stats_view.filter_str.length;
      // Compute the best match
      for (let i = 0; i < tabcontent.length; i++) {
        const full_str = tabcontent[i].children[0].original_name + "\n" +
                         tabcontent[i].children[1].original_name;
        bmatch = Math.min(
            bmatch, TextFilter(full_str, stats_view.filter_str, stats_view.filter_str.length + 10));
      }
      for (let i = 0; i < tabcontent.length; i++) {
        const full_str = tabcontent[i].children[0].original_name + "\n" +
                         tabcontent[i].children[1].original_name;
        let match = TextFilter(full_str, stats_view.filter_str, stats_view.filter_str.length + 10,
                               "<b><font color='red'>", "</font></b>");
        if (match.split > bmatch + 1) {
          tabcontent[i].style.display = "none";
        } else {
          tabcontent[i].style.display = "";
          let split = match.text.split("\n");
          tabcontent[i].children[0].innerHTML = split[0];
          tabcontent[i].children[1].innerHTML = split[1];
        }
      }
      stats_view.colorRows();
    };

    stats_view.input_elem.onkeyup = stats_view.showCodelet;

    // Create the table to rend the statistics
    stats_view.table_div = document.createElement("table");
    stats_view.appendChild(stats_view.table_div);

    let footer = document.createElement("div");
    footer.style.bottom = "0px";
    footer.style.position = "sticky";
    footer.style.backgroundColor = "#ddd";
    stats_view.appendChild(footer);
    const sort_style = ["sorting_desc", "sorting", "sorting_asc"];
    {
      let row = document.createElement("tr");
      stats_view.titles = [
          "Node", "Codelet", "#Ticks", "Average time [ms]", "median", "90 percentile", "max",
          "Frequency [Hz]", "Load", "%late"
      ];
      for (let id in stats_view.titles) {
        let style = document.createElement("style");
        stats_view.appendChild(style);
        style.innerHTML = "";
        let th = document.createElement("th");
        th.classList.add("tabcol"+id);
        th.classList.add(sort_style[1]);
        th.appendChild(document.createTextNode(stats_view.titles[id]));
        th.style.cursor = "pointer";
        th.onclick = function(evt) {
          row.childNodes[stats_view.sort_col].classList.remove(sort_style[1 + stats_view.sort_dir]);
          if (stats_view.sort_col == id) {
            stats_view.sort_dir *= -1;
          } else {
            stats_view.sort_col = id;
            stats_view.sort_dir = 1;
          }
          row.childNodes[stats_view.sort_col].classList.add(sort_style[1 + stats_view.sort_dir]);
          stats_view.sort();
        };
        row.appendChild(th);
        let checkbox = document.createElement("input");
        checkbox.setAttribute("type", "checkbox");
        checkbox.checked = getConfig("__statistics-col_"+id, 'true') == 'true';
        checkbox.onchange = function() {
          saveConfig("__statistics-col_"+id, checkbox.checked);
          if (checkbox.checked) {
            style.innerHTML = "";
          } else {
            style.innerHTML = ".tabcol" + id + " {display: none}";
          }
        };
        checkbox.onchange();
        let label = document.createElement("label");
        label.innerHTML = stats_view.titles[id];
        label.appendChild(checkbox);
        checkbox.style.margin = "5px";
        label.style.margin = "5px";
        footer.appendChild(label);
      }
      row.childNodes[stats_view.sort_col].classList.add(sort_style[1 + stats_view.sort_dir]);
      stats_view.table_div.appendChild(row);
    }
    // Create a new window with a callback to call resize() on the cytoscape object.
    WindowManager().createWindow(
        stats_view,
        "Statistics",
        {
          hide: true,
          onmaximize: function() { UpdateStatistics(stats_view.data); }
        });
    // Remove the legend
    stats_view.parentNode.removeChild(stats_view.parentNode.lastChild);
  }
  return stats_view;
}

// Reset the statistics widget.
function ResetStatisticsView() {
  // Delete the current view;
  const win = WindowManager().getWindow("Statistics");
  if (win) WindowManager().deleteWindow(win);
}
