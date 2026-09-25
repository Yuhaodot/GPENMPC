# Initial environment

`initial_environment.mat` contains the numeric `initialEnvironment` frame
used to initialize the plant-environment encoder. It provides the wind,
payload, reference and timing fields for the 232-byte ENV2 message.

Run `test_initial_environment` from `source/hil/tools` to check the frame
and its encoding.
