#include <iostream>
#include <fstream>
#include <string>

void generateConfigFile(
    const std::string& filename,
    const std::string& hand_geometry_filename,
    const std::string& image_geometry_filename,
    const std::string& weights_file,
    int voxelize,
    double voxel_size,
    int remove_outliers,
    const std::string& workspace,
    const std::string& camera_position,
    int sample_above_plane,
    int num_samples,
    int num_threads,
    double nn_radius,
    int num_orientations,
    int num_finger_placements,
    int hand_axes,
    int deepen_hand,
    int friction_coeff,
    int min_viable,
    double min_aperture,
    double max_aperture,
    const std::string& workspace_grasps,
    int filter_approach_direction,
    const std::string& direction,
    double thresh_rad,
    int min_inliers,
    int num_selected,
    int plot_normals,
    int plot_samples,
    int plot_candidates,
    int plot_filtered_candidates,
    int plot_valid_grasps,
    int plot_clustered_grasps,
    int plot_selected_grasps
) {
    std::ofstream cfg_file(filename);
    if (!cfg_file.is_open()) {
        std::cerr << "Error: Unable to create file: " << filename << std::endl;
        return;
    }

    cfg_file << "# Path to config file for robot hand geometry\n";
    cfg_file << "hand_geometry_filename = " << hand_geometry_filename << "\n\n";

    cfg_file << "# Path to config file for volume and image geometry\n";
    cfg_file << "image_geometry_filename = " << image_geometry_filename << "\n\n";

    cfg_file << "# Path to directory that contains neural network parameters\n";
    cfg_file << "weights_file = " << weights_file << "\n\n";

    cfg_file << "# Preprocessing of point cloud\n";
    cfg_file << "voxelize = " << voxelize << "\n";
    cfg_file << "voxel_size = " << voxel_size << "\n";
    cfg_file << "remove_outliers = " << remove_outliers << "\n";
    cfg_file << "workspace = " << workspace << "\n";
    cfg_file << "camera_position = " << camera_position << "\n";
    cfg_file << "sample_above_plane = " << sample_above_plane << "\n\n";

    cfg_file << "# Grasp candidate generation\n";
    cfg_file << "num_samples = " << num_samples << "\n";
    cfg_file << "num_threads = " << num_threads << "\n";
    cfg_file << "nn_radius = " << nn_radius << "\n";
    cfg_file << "num_orientations = " << num_orientations << "\n";
    cfg_file << "num_finger_placements = " << num_finger_placements << "\n";
    cfg_file << "hand_axes = " << hand_axes << "\n";
    cfg_file << "deepen_hand = " << deepen_hand << "\n";
    cfg_file << "friction_coeff = " << friction_coeff << "\n";
    cfg_file << "min_viable = " << min_viable << "\n\n";

    cfg_file << "# Filtering of candidates\n";
    cfg_file << "min_aperture = " << min_aperture << "\n";
    cfg_file << "max_aperture = " << max_aperture << "\n";
    cfg_file << "workspace_grasps = " << workspace_grasps << "\n\n";

    cfg_file << "# Filtering of candidates based on their approach direction\n";
    cfg_file << "filter_approach_direction = " << filter_approach_direction << "\n";
    cfg_file << "direction = " << direction << "\n";
    cfg_file << "thresh_rad = " << thresh_rad << "\n\n";

    cfg_file << "# Clustering of grasps\n";
    cfg_file << "min_inliers = " << min_inliers << "\n\n";

    cfg_file << "# Grasp selection\n";
    cfg_file << "num_selected = " << num_selected << "\n\n";

    cfg_file << "# Visualization\n";
    cfg_file << "plot_normals = " << plot_normals << "\n";
    cfg_file << "plot_samples = " << plot_samples << "\n";
    cfg_file << "plot_candidates = " << plot_candidates << "\n";
    cfg_file << "plot_filtered_candidates = " << plot_filtered_candidates << "\n";
    cfg_file << "plot_valid_grasps = " << plot_valid_grasps << "\n";
    cfg_file << "plot_clustered_grasps = " << plot_clustered_grasps << "\n";
    cfg_file << "plot_selected_grasps = " << plot_selected_grasps << "\n";

    cfg_file.close();
    std::cout << "Configuration file generated: " << filename << std::endl;
}

int main() {
    std::string filename = "generated_config.cfg";
    generateConfigFile(
        filename,
        "/workspace/ros2-workspace/call_m_arm_ws/src/call_m_arm/gpd/cfg/hand_geometry.cfg",
        "/workspace/ros2-workspace/call_m_arm_ws/src/call_m_arm/gpd/cfg/image_geometry_15channels.cfg",
        "/workspace/ros2-workspace/call_m_arm_ws/src/call_m_arm/gpd/models/lenet/15channels/params/",
        1, 0.003, 0, "-1.0 1.0 -1.0 1.0 -1.0 1.0", "0 0 0", 1,
        500, 4, 0.01, 8, 10, 2, 1, 20, 6,
        0.0, 0.085, "-1 1 -1 1 -1 1",
        1, "-1 0 0", 0.75, 0,
        1, 0, 0, 0, 0, 0, 0, 0
    );
    return 0;
}
