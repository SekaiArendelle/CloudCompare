# Pipe Centerline Plugin Test

This directory contains test cases and sample data for the qPipeCenterline plugin.

## Test Scenarios

1. **Simple Straight Pipe**
   - Generate synthetic straight pipe point cloud
   - Test basic centerline extraction
   - Verify accuracy of extracted centerline

2. **Curved Pipe**
   - Generate synthetic curved pipe point cloud
   - Test centerline extraction with curvature
   - Verify smoothness of extracted centerline

3. **Branched Pipe**
   - Generate synthetic pipe with branches
   - Test branch detection and handling
   - Verify multiple centerline segments

4. **Noisy Data**
   - Add noise to synthetic pipe data
   - Test robustness of preprocessing
   - Verify centerline extraction quality

5. **Real Data**
   - Use real scanned pipe data
   - Test performance on real-world scenarios
   - Validate results against ground truth

## Usage

1. Load the plugin in CloudCompare
2. Select a point cloud representing a pipe
3. Run the plugin and adjust parameters
4. Verify the extracted centerlines

## Parameters Guidelines

- **Estimated Pipe Radius**: Start with approximate pipe radius
- **Voxel Size**: Use 1-5% of pipe radius for good balance
- **Curvature Threshold**: Lower values for smoother pipes
- **Branch Detection**: Enable for pipes with bifurcations