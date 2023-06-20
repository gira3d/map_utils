# Map Utils

## Build and Run Unit Tests

1. Pull the test input files

```shell
git lfs install
git lfs fetch
git lfs pull
```

2. Compile the unit tests

`catkin build --make-args tests -- map_utils`

3. Run the tests

`rosrun map_utils test_incremental_pointcloud`

`rosrun map_utils test_intersections`

4. To generate the unit test data, open the unit test files and change

`#define GENERATE_TEST_RESULTS 0`

to

`#define GENERATE_TEST_RESULTS 1`
