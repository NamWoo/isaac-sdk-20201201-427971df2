'''
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
'''

import sys
import numpy as np
import os.path
import matplotlib.pyplot as plt

def printUsage(errmsg = ""):
    if len(errmsg): print("Error: " + errmsg)
    print()
    print("Usage: epnp_eval_plots.py </path/to/eval/files>")
    print()
    sys.exit()
    return

def parseArgs():
    args = sys.argv
    if len(args)==1 : printUsage()
    evalPath = args[1]
    return evalPath

# list files of given extension in a directory
def listFiles(inpath,extension):
    allfiles = os.listdir(inpath)
    outfiles = list()
    for s in allfiles:
        filename,ext  = os.path.splitext(s)
        if ext==extension:
            outfiles.append(filename)
    return outfiles

class ParseError(Exception):
    def __init__(self, msg):
        self.msg = msg
    pass

def nextValidLine(file): # read line but skip empty lines and lines starting by #
    while 1:
        line = file.readline()
        if not line: return (None,[])
        line = line.rstrip()
        if len(line)>0 and line[0]!='#':
            words = line.split()
            if len(words):
                return (line,words)

def asint(txt, name):
    try: x = int(txt)
    except: raise ParseError("unable to parse %s" % name)
    return(x)

def asfloat(txt, name):
    try: x = float(txt)
    except: raise ParseError("unable to parse %s" % name)
    return(x)

def parseEvalFile(filename):
    file = open(filename,"r")

    # parse file signature and benchmark type
    line,words = nextValidLine(file)
    if not line or len(words)!=2: raise ParseError("not an epnp benchmark file")
    if words[0]!="epnp_benchmark": raise ParseError("not an epnp benchmark file")
    if words[1]!="accuracy": raise ParseError("unsupported benchmark type")
    line,words = nextValidLine(file)
    if not line or len(words)==0: raise ParseError("unable to parse benchmark parameters")

    # parse benchmark parameters (they are in a single line)
    data = {}
    while len(words):
        data[words[0]] = asfloat(words[1],words[0])
        del words[0:2]

    # parse data matrix as floats
    mat = []
    while 1:
        (line,words) = nextValidLine(file)
        if not line: break
        for index, value in enumerate(words):
            try: words[index] = float(value)
            except: raise ParseError("unable to parse a line of values")
        mat.append(words)
    mat = np.asarray(mat)
    data['matrix'] = mat
    if data['matrix'].shape[1]!=14: raise ParseError("unexpected number of values in matrix")
    return data

def createTitle(data):
    title = ""
    if 'near' in data:
        title = "Non-planar, depth in [%.1f,%.1f] m\n%dx%d px, f=%.1f px, %d runs" \
        % (data['near'],data['far'],data['width'],data['height'],data['focal'],data['runs'])
    elif 'plane_depth' in data:
        title = "Plane at %.1f meters, angle=%.1f deg, non-planarity=%.1f cm\n%dx%d px, f=%.1f px, %d runs" \
        % (data['plane_depth'],data['plane_angle'],100*data['plane_deviation'],data['width'],data['height'],data['focal'],data['runs'])
    return title

def generateMeanErrorVersusNoisePlots(filename, data):
    mat = data["matrix"]
    title = createTitle(data)

    xlabel = 'Image noise $\sigma$ (pixels)'
    selected_points = [6, 8, 12, 20, 50]
    styles = ['k*-', 'ro-', 'g+-','m^-','bs-', 'cx-']
    fig = plt.figure(figsize=(1,1))
    plt.subplot(121)
    ax = fig.gca()
    for i in range(0,len(selected_points)):
        idx = mat[:,0]==selected_points[i]
        if len(idx)==0: continue
        noise_sigma = mat[idx,1]
        rot_error_mean = mat[idx,9]
        label = "%d points" % selected_points[i]
        h = ax.plot(noise_sigma,rot_error_mean, styles[i], label=label)

    ax.legend(loc=2)
    ax.grid(color=[0.9,0.9,0.9], linestyle='-', linewidth=1,zorder=0)
    ax.set_xlabel(xlabel)
    ax.set_ylabel('Mean Rotation Error (degrees)')
    ax.set_ylim([0,3.1])
    ax.set_title(title,fontsize=10)

    plt.subplot(122)
    ax = fig.gca()
    for i in range(0,len(selected_points)):
        idx = mat[:,0]==selected_points[i]
        noise_sigma    = mat[idx,1]
        pos_error_mean = mat[idx,11]
        label = "%d points" % selected_points[i]
        ax.plot(noise_sigma,pos_error_mean, styles[i], label=label)

    ax.legend(loc=2)
    ax.grid(color=[0.9,0.9,0.9], linestyle='-', linewidth=1,zorder=0)
    ax.set_xlabel(xlabel)
    ax.set_ylabel('Mean Position Error (meters)')
    ax.set_ylim([0,0.3])
    ax.set_title(title,fontsize=10)

    fig.set_dpi(100)
    fig.set_size_inches(12,4)
    out_file = filename + ".png"
    print("Writing", out_file)
    plt.savefig(out_file, bbox_inches='tight')
    #plt.show()

def generateMeanErrorVersusPointsPlots(filename, data):
    mat = data["matrix"]
    title = createTitle(data)

    noise_sigmas = np.unique(mat[:,1])
    noise_sigmas  = noise_sigmas[noise_sigmas>0]
    #print("Noise: ", list(noise_sigmas))

    styles = ['cx-', 'bs-', 'm^-', 'g+-', 'ro-', 'k*-', 'cx:', 'bs:', 'm^:', 'g+:', 'ro:', 'k*:']
    fig = plt.figure(figsize=(1,1))
    plt.subplot(121)
    ax = fig.gca()
    for i in range(0,len(noise_sigmas)):
        idx = mat[:,1]==noise_sigmas[i]
        if len(idx)==0: continue
        num_points = mat[idx,0]
        rot_error_mean = mat[idx,9]
        label = "$\sigma$=%.1f px" % noise_sigmas[i]
        h = ax.plot(num_points,rot_error_mean, styles[i], label=label)

    ax.legend(loc=1)
    ax.grid(color=[0.9,0.9,0.9], linestyle='-', linewidth=1, zorder=0)
    ax.set_xlabel('#points')
    ax.set_xscale('log')
    ax.set_ylabel('Mean Rotation Error (degrees)')
    ax.set_ylim([0,3.1])
    ax.set_title(title,fontsize=10)

    plt.subplot(122)
    ax = fig.gca()
    for i in range(0,len(noise_sigmas)):
        idx = mat[:,1]==noise_sigmas[i]
        if len(idx)==0: continue
        num_points = mat[idx,0]
        pos_error_mean = mat[idx,11]

        label = "$\sigma$=%.1f px" % noise_sigmas[i]
        h = ax.plot(num_points,pos_error_mean, styles[i], label=label)

    ax.legend(loc=1)
    ax.grid(color=[0.9,0.9,0.9], linestyle='-', linewidth=1, zorder=0)
    ax.set_xlabel('#points')
    ax.set_xscale('log')
    ax.set_ylabel('Mean Position Error (meters)')
    ax.set_ylim([0,0.3])
    ax.set_title(title,fontsize=10)

    fig.set_dpi(100)
    fig.set_size_inches(12,4)
    out_file = filename + ".png"
    print("Writing", out_file)
    plt.savefig(out_file, bbox_inches='tight')
    # plt.show()

if __name__ == "__main__":

    np.set_printoptions(precision=4)
    np.set_printoptions(suppress=True)

    inpath = parseArgs()

    filenames = [\
        'epnp-eval-f700-nonplanar.txt',\
        'epnp-eval-f700-planar-5m-fronto.txt',\
        'epnp-eval-f700-planar-5m-slanted.txt',\
        'epnp-eval-f700-quasiplanar-5m-slanted.txt',\
        ]

    #ext = '.eval'
    #files = listFiles(inpath,ext)
    #print("%d '%s' files found in directory inpath" % (len(files), ext))

    for filename in filenames:
        filename = inpath + os.sep + filename
        if not os.path.isfile(filename):
            print("File not found:",filename)
            continue
        filename,ext  = os.path.splitext(filename)

        # parse benchmark data file
        print("Parsing", filename + ext)
        try: data = parseEvalFile(filename+ext)
        except ParseError as err:
           print("Parsing error %s: %s" % (filename,err.msg))
           sys.exit()

        # generate plots
        generateMeanErrorVersusNoisePlots(filename + '-erravg-noise', data)
        generateMeanErrorVersusPointsPlots(filename + '-erravg-numpts', data)
        #break
