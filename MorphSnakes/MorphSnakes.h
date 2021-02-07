#include <algorithm>
#include <iostream>
#include "morphsnakes.h"
#include <fstream>

//#include "/home/joshua/Dokumente/Bachelor/lpng1637/png.h"
#include <png.h>

#include <zlib.h>
#define cimg_display 0
#include "/home/joshua/Dokumente/Bachelor/CImg_latest/CImg-2.9.1_pre042420/CImg.h"
#include <variables.h>

const string filepath = "MorphSnakes/images/rs_" + path + ".png";
const string GACoutputPath = "MorphSnakes/output/GAC/rs_" + path + ".png";
const string GACoutputPathStart = "MorphSnakes/output/GAC/rs_" + path + "_start.png";
const string ACWEoutputPath = "MorphSnakes/output/ACWE/rs_" + path + ".png";
const string ACWEoutputPathStart = "MorphSnakes/output/ACWE/rs_" + path + "_start.png";

ofstream dataFile;

namespace ms = morphsnakes;
using namespace cimg_library;
using namespace std;