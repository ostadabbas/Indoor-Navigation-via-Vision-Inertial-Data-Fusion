clc
clear all
close all

load('C:\Users\Amir\Downloads\RGB-d\imp-pdf\m.mat');

[acc,ta]=m.accellog;
[gyr,tg]=m.orientlog;
[mag,tm]=m.magfieldlog;