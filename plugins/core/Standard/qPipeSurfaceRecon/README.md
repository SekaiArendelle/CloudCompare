# qPipeSurfaceRecon Plugin

This CloudCompare plugin provides one-click surface reconstruction optimized specifically for pipe point clouds.

## Features

- **Automated Pipe-Optimized Workflow**: Combines multiple PCL algorithms specifically tuned for pipe-like structures
- **One-Click Operation**: Simple interface - just select your point cloud and click
- **Pipe-Specific Enhancements**:
  - Statistical outlier removal for noisy pipe scans
  - Moving Least Squares (MLS) smoothing and upsampling
  - Optimized Poisson surface reconstruction parameters for cylindrical structures
  - Automatic normal estimation if not present

## Requirements

- CloudCompare 2.13+
- PCL (Point Cloud Library) 1.9+
- Qt 5.12+

## Installation

1. Build the plugin with CMake:
   ```bash
   cmake -DPLUGIN_STANDARD_QPIPE_SURFACE_RECON=ON ..
   cmake --build .
   ```

2. The plugin will be automatically loaded by CloudCompare when placed in the plugins directory.

## Usage

1. Load your pipe point cloud into CloudCompare
2. Select the point cloud in the DB tree
3. Click on the "Pipe Surface Reconstruction" icon in the plugin toolbar
4. Wait for the reconstruction to complete
5. The resulting mesh will be added to the database

## Algorithm Details

The plugin implements the following pipeline:
1. **Statistical Outlier Removal**: Removes noise common in pipe scanning
2. **Normal Estimation**: Computes normals if not already present
3. **MLS Smoothing**: Smooths and upsamples the point cloud for better surface quality
4. **Poisson Reconstruction**: Generates the final mesh with parameters optimized for pipes

## Parameters

The plugin uses optimized default parameters suitable for most pipe point clouds:
- Octree depth: 8 (balance between detail and performance)
- Point weight: 4.0 (higher for pipe surfaces)
- MLS search radius: 1cm (adjusts based on typical pipe dimensions)
- Upsampling: Enabled for better surface continuity

## Troubleshooting

- **Not enough points**: Ensure your point cloud has at least 100 points
- **Reconstruction fails**: Check if the point cloud has sufficient density and coverage
- **Poor results**: Try adjusting the MLS search radius based on your pipe diameter