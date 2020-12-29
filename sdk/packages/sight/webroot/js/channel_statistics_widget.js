/*
Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
// Helper class to record new messages and compute statistics about the various channel.
// It helps keeping the list of most recent messages, and keep track of their size avoid recomputing
// from scratch each time.
class StatisticsRecorder {
  constructor() {
    // Contains the list of channels we received from the robot
    this.channels = {};
    // Keep at least that much data in ms.
    this.time_window = 5000.0;
  }

  // Add a new message to the queue:
  // - fullname: app/node/codelet/channel_name or ///channel_name for isaac messages as PoseTree
  // - size: the size of the messages in bytes
  // - type: type of the message: Config/PoseTree/Replay/Image/Plot/...
  // - status: helper to access whether or not the channel is currently active.
  addMessage(fullname, size, type, status = {status_: true}) {
    // For new channel we need to create a new entry.
    if (!this.channels[fullname]) {
      const split = fullname.split("/");
      if (split.length < 4) return;
      let data = {
        node_name: split[1],
        codelet_name: split[2],
        channel_name: split[3],
        size: 0,
        total_size: 0,
        num_messages: 0,
        status, status,
        list: []
      };
      this.channels[fullname] = data;
    }
    // Update the channel
    let channel = this.channels[fullname];
    channel.type = type;
    channel.size += size;
    channel.total_size += size;
    channel.num_messages++;
    const now = Date.now();
    channel.list.push([now, size]);
    // Make sure we don;t accumulate too much data in case the channel statistics widget is not
    // visible.
    if (channel.list[0][0] < 4 * this.time_window + now) {
      this.refresh(channel, now - this.time_window);
    }
  }

  // Refreshes the channel by deleting data older than start_time.
  refresh(channel, start_time) {
    while (channel.list.length > 0 && channel.list[0][0] < start_time) {
      let msg = channel.list.shift();
      channel.size -= msg[1];
    }
  }

  // Returns the current number of messages.
  getNumberMessages(channel) {
    return channel.list.length;
  }

  // Returns the sum of the size of all the current messages.
  getCurrentMessageSize(channel) {
    return channel.size;
  }

  // Returns the average size of all the messages since the beginning.
  getAverageMessageSize(channel) {
    return channel.total_size / channel.num_messages;
  }

  // Returns whether or not it's an active channel.
  isChannelActive(channel) {
    return channel.status.status_;
  }
};
let statistics_recorder_ = new StatisticsRecorder();

// Updates the channel statistics widget.
function UpdateChannelStatistics() {
  let view = GetChannelStatisticsView();
  // If the view is not visible let's not do anything.
  if (!view.isVisible()) return;

  // Converts a size into a mb or kb if needed.
  let getSize = function(value) {
    if (value > 1024 * 1024) {
      return (value/(1024 * 1024)).toFixed(1) + " mb";
    } else if (value > 1024) {
      return (value/1024.0).toFixed(1) + " kb";
    } else {
      return value + " b";
    }
  }

  // Check if there was a new channel
  let new_channel = false;
  // We compute the statistics over the last 3s
  const kDuration = 3.0;
  const start_time = Date.now() - kDuration * 1000;
  // Loop through all the channels
  for (const fullname in statistics_recorder_.channels) {
    const channel = statistics_recorder_.channels[fullname];

    statistics_recorder_.refresh(channel, start_time);
    try {
      // Add a new row if needed.
      if (!view.list_channels[fullname]) {
        // If this is a new channel, we create a new row.
        let row = document.createElement("tr");
        row.classList.add("tabcontent");
        for (let i = 0; i < view.titles.length; i++) {
          let td = document.createElement("td");
          td.classList.add("cs_tabcol"+i);
          row.appendChild(td);
          view.table_div.appendChild(row);
        }
        view.list_channels[fullname] = row;
        new_channel = true;
        row.children[0].original_name = channel.node_name;
        row.children[1].original_name = channel.codelet_name;
        row.children[2].original_name = channel.channel_name;
        row.children[7].className += " material-icons-stats";
      }
      // Update the row.
      let row = view.list_channels[fullname];
      row.children[3].rawValue = statistics_recorder_.getNumberMessages(channel) / kDuration;
      row.children[3].innerHTML = row.children[3].rawValue.toFixed(1);
      row.children[4].rawValue = statistics_recorder_.getCurrentMessageSize(channel) / kDuration;
      row.children[4].innerHTML = getSize(row.children[4].rawValue.toFixed(1)) + "/s";
      row.children[5].rawValue = statistics_recorder_.getAverageMessageSize(channel);
      row.children[5].innerHTML = getSize(row.children[5].rawValue.toFixed(0));
      row.children[6].innerHTML = channel.type;
      row.children[7].rawValue = statistics_recorder_.isChannelActive(channel) ? 1 : 0;
      row.children[7].innerHTML = row.children[7].rawValue ? "check" : "";
    } catch (e) {
      console.error(e);
    }
  }
  if (new_channel) view.showChannels();
  view.sort();
}

// Returns the div containing the channel statistics view, if it does not exist yet create it.
function GetChannelStatisticsView() {
  let stats_view = document.getElementById("__win-channel-statistics-view");
  if (stats_view === null || stats_view === undefined) {
    saveConfig("__channel_statistics_widget", true);
    // Create the new widget
    stats_view = document.createElement("div");
    stats_view.style.display = "block";
    stats_view.id = "__win-channel-statistics-view";
    // Create the search input
    let top_bar = document.createElement("div");
    stats_view.appendChild(top_bar);
    top_bar.classList.add("tab");
    top_bar.style.display = "inline-flex";
    let search_div = document.createElement("div");
    search_div.style.width = "100%";
    search_div.style.minWidth = "215px";
    top_bar.appendChild(search_div);
    stats_view.input_elem = document.createElement("input");
    stats_view.input_elem.type = "text";
    stats_view.input_elem.placeholder = "Filter";
    search_div.appendChild(stats_view.input_elem);
    // Parameters of the widget
    stats_view.list_channels = {};  // Dictionary to easily access the row of a given channel
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
        if (stats_view.sort_col < 3 || stats_view.sort_col == 6) {
          // Sort lexicographically by default
          const al = a.children[stats_view.sort_col].innerHTML.toLowerCase();
          const bl = b.children[stats_view.sort_col].innerHTML.toLowerCase();
          if (al == bl) return 0;
          return al < bl ? -stats_view.sort_dir : stats_view.sort_dir;
        } else {
          // Sort ascending order by default
          return (parseFloat(a.children[stats_view.sort_col].rawValue) -
                  parseFloat(b.children[stats_view.sort_col].rawValue)) * stats_view.sort_dir;
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
    stats_view.showChannels = function(evt) {
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
                         tabcontent[i].children[1].original_name + "\n" +
                         tabcontent[i].children[2].original_name;
        bmatch = Math.min(
            bmatch, TextFilter(full_str, stats_view.filter_str, stats_view.filter_str.length + 10));
      }
      for (let i = 0; i < tabcontent.length; i++) {
        const full_str = tabcontent[i].children[0].original_name + "\n" +
                         tabcontent[i].children[1].original_name + "\n" +
                         tabcontent[i].children[2].original_name;
        let match = TextFilter(full_str, stats_view.filter_str, stats_view.filter_str.length + 10,
                               "<b><font color='red'>", "</font></b>");
        if (match.split > bmatch + 1) {
          tabcontent[i].style.display = "none";
        } else {
          tabcontent[i].style.display = "";
          let split = match.text.split("\n");
          tabcontent[i].children[0].innerHTML = split[0];
          tabcontent[i].children[1].innerHTML = split[1];
          tabcontent[i].children[2].innerHTML = split[2];
        }
      }
      stats_view.colorRows();
    };

    stats_view.input_elem.onkeyup = stats_view.showChannels;

    // Create the table to rend the statistics
    stats_view.table_div = document.createElement("table");
    stats_view.appendChild(stats_view.table_div);

    let footer = document.createElement("div");
    footer.style.bottom = "0px";
    footer.style.position = "sticky";
    footer.style.backgroundColor = "#ddd";
    stats_view.appendChild(footer);
    const sort_style = ["sorting_desc", "sorting", "sorting_asc"];

    let row = document.createElement("tr");
    stats_view.titles = [
        "Node", "Codelet", "Channel", "Frequency [Hz]", "Bandwidth", "Data size", "Type", "Active"
    ];
    for (let id in stats_view.titles) {
      let style = document.createElement("style");
      stats_view.appendChild(style);
      style.innerHTML = "";
      let th = document.createElement("th");
      th.classList.add("cs_tabcol"+id);
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
      checkbox.checked = getConfig("__channel_statistics-col_"+id, 'true') == 'true';
      checkbox.onchange = function() {
        saveConfig("__channel_statistics-col_"+id, checkbox.checked);
        if (checkbox.checked) {
          style.innerHTML = "";
        } else {
          style.innerHTML = ".cs_tabcol" + id + " {display: none}";
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

    // Update the channels data at 4Hz
    const interval_id = setInterval(UpdateChannelStatistics, 250);
    // Create a new window with a callback to call resize() on the cytoscape object.
    WindowManager().createWindow(stats_view, "Channel Statistics",
                                 {close: true,
                                  onclose: function() {
                                    clearInterval(interval_id);
                                    saveConfig("__channel_statistics_widget", false);
                                  },
                                  onmaximize: UpdateChannelStatistics});
    // Remove the legend
    stats_view.parentNode.removeChild(stats_view.parentNode.lastChild);
  }
  return stats_view;
}

// Creates the view if it does not exist, and displays if it's currently hidden.
function CreateChannelStatisticsView() {
  UpdateChannelStatistics();
  const win = WindowManager().getWindow("Channel Statistics");
  if (!win.check.checked) {
    WindowManager().maximizeWindow(win);
  }
}
