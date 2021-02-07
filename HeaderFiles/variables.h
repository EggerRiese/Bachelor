#pragma once
#ifndef VARIABLES_H_
#define VARIABLES_H_

#include <string>

using namespace std;


const int KeyFrames[15] = {5, 17, 29, 41, 53, 
65, 77, 89, 101, 113, 
125, 137, 149, 161, 173};

const int rsImageWidth = 1280;
const int rsImageHeight = 720;

const int BoundingBox[12] = {520, 196, 255, 197,
791, 197, 59, 210,
596, 180, 387, 187};

//Bread
const int boundingBoxStartX = 544;
const int boundingBoxStartY = 348;
const int boundingBoxLengthX = 310;
const int boundingBoxLengthY = 221;

//Banana
// const int boundingBoxStartX = 365;
// const int boundingBoxStartY = 334;
// const int boundingBoxLengthX = 213;
// const int boundingBoxLengthY = 396;

//cup_drink
// const int boundingBoxStartX = 863;
// const int boundingBoxStartY = 244;
// const int boundingBoxLengthX = 269;
// const int boundingBoxLengthY = 228;



//const string path = "broetchen_saft";
// const string path = "broetchen_kartoffel";
// const string path = "broetchen_kartoffel_licht";
// const string path = "chilli_sin_carne";
// const string path = "gebratene_nudeln";
// const string path = "hack_lauch_suppe";
// const string path = "nudel_brokoli";
// const string path = "pizza_ganz";
// const string path = "pizza_geschnitten";
// const string path = "spinat_leberkaese_kartoffelbrei";
// const string path = "wurst_kartoffel_gemuese_clean";
 const string path = "wurst_kartoffel_gemuese_dirty";

#endif