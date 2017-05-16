# CarND_Extended_Kalman_Filter_Project
Instructions on how to run the extended kalman filter project:

1)  make a directory called build with mkdir build and cd build to go to the build directory

2)  use cmake and make to create an executable file called UnscentedKF.exe with the following command: cmake .. && make

3) run the input data files: ./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output1.txt > log1; ./ExtendedKF ../data/sample-laser-radar-measurement-data-2.txt output2.txt > log2;

4) open up each log file to see the results: more log1; more log2;
