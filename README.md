# Dynamic Based Estimator for UAVs withReal-time Identification Using DNN andthe Modified Relay Feedback Test.

[![Dynamic Based Estimator for UAVs withReal-time Identification Using DNN andthe Modified Relay Feedback Test](screencap.jpg)](https://youtu.be/vNNliifjPig)

This is the code for the 2021 IEEE paper: [Dynamic Based Estimator for UAVs withReal-time Identification Using DNN andthe Modified Relay Feedback Test](https://ieeexplore.ieee.org/): 

## Running examples

### Rotational Dynamic Extended Kalman Filter:

After running the DNN-MRFT Pipeline, update the parameters in "convertWorkbookToFunctionsRDEKF" and run the script
**Run the script**:
```
convertWorkbookToFunctionsRDEKF.m
```
This will create matlab function files inside the "RDEKF_files" folder. Use these files to update the Simulink block inside "RDEKF.slx" accordingely.

### Translational Dynamic Extended Kalman Filter:

After running the DNN-MRFT Pipeline, update the parameters in "convertWorkbookToFunctionsTDEKF" and run the script
**Run the script**:
```
convertWorkbookToFunctionsTDEKF.m
```
This will create matlab function files inside the "TDEKF_files" folder. Use these files to update the Simulink block inside "TDEKF.slx" accordingely.

## Additional Notes

Feel free to contact the repository owner if you need additional information regarding the code or the algorithm.
