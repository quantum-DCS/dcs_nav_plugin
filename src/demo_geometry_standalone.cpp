/**
 * @file demo_geometry_standalone.cpp
 * @brief Standalone demo to visualize convex polygon decomposition from a map image.
 * 
 * Algorithm (Strictly following design doc):
 * 1. Binary threshold: obstacles (dark) -> white
 * 2. Dilate BEFORE findContours (R >= epsilon)
 * 3. findContours with RETR_LIST (not EXTERNAL!)
 * 4. approxPolyDP (epsilon = 0.02m in world coords)
 * 5. Hertel-Mehlhorn convex decomposition
 * 
 * Usage: ./demo_geometry_standalone <map_pgm_path> [output_path] [resolution]
 * Example: ./demo_geometry_standalone house_map.pgm output.png 0.05
 */
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <list>
#include <algorithm>
#include "polypartition.h"

// Minimal Polygon type for the demo
struct Point2D { double x, y; };
using Polygon = std::vector<Point2D>;

/**
 * @brief Apply Hertel-Mehlhorn convex decomposition to a single contour.
 * Falls back to Convex Hull if decomposition fails.
 */
std::vector<Polygon> decomposeConvex(const std::vector<cv::Point>& contour, double epsilon_pix) {
    // 1. Simplify contour with approxPolyDP
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, epsilon_pix, true);
    
    if (approx.size() < 3) {
        // Too small, use convex hull of original contour
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        if (hull.size() >= 3) {
            Polygon poly;
            for (const auto& pt : hull) {
                poly.push_back({(double)pt.x, (double)pt.y});
            }
            return {poly};
        }
        return {};
    }

    // 2. Ensure CCW orientation (Polypartition requirement)
    double area = cv::contourArea(approx, true);
    if (area < 0) {
        std::reverse(approx.begin(), approx.end());
    }

    // 3. Convert to TPPLPoly
    TPPLPoly tppl_poly;
    tppl_poly.Init(approx.size());
    for (size_t i = 0; i < approx.size(); ++i) {
        tppl_poly[i].x = approx[i].x;
        tppl_poly[i].y = approx[i].y;
    }
    tppl_poly.SetHole(false);
    tppl_poly.SetOrientation(TPPL_ORIENTATION_CCW);

    // 4. Try Hertel-Mehlhorn decomposition
    TPPLPartition pp;
    std::list<TPPLPoly> input_polys, result_polys;
    input_polys.push_back(tppl_poly);
    
    if (!pp.ConvexPartition_HM(&input_polys, &result_polys)) {
        // Decomposition failed, fallback to convex hull
        std::vector<cv::Point> hull;
        cv::convexHull(contour, hull);
        if (hull.size() >= 3) {
            Polygon poly;
            for (const auto& pt : hull) {
                poly.push_back({(double)pt.x, (double)pt.y});
            }
            return {poly};
        }
        return {};
    }

    // 5. Convert back
    std::vector<Polygon> output;
    for (auto& p : result_polys) {
        if (p.GetNumPoints() < 3) continue;
        Polygon poly;
        for (long i = 0; i < p.GetNumPoints(); ++i) {
            poly.push_back({p[i].x, p[i].y});
        }
        output.push_back(poly);
    }
    return output;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <map_pgm_path> [output_path] [resolution_m]" << std::endl;
        return 1;
    }

    std::string map_path = argv[1];
    std::string output_path = (argc > 2) ? argv[2] : "demo_output.png";
    double resolution = (argc > 3) ? std::stod(argv[3]) : 0.05; // Default 0.05m/pixel

    // Parameters from design doc
    double epsilon_m = 0.02; // approxPolyDP epsilon in meters
    double dilate_radius_m = epsilon_m; // Dilation radius >= epsilon
    
    int epsilon_pix = std::max(1, (int)(epsilon_m / resolution));
    int dilate_pix = std::max(1, (int)(dilate_radius_m / resolution));

    std::cout << "Parameters:" << std::endl;
    std::cout << "  Resolution: " << resolution << " m/pixel" << std::endl;
    std::cout << "  Epsilon: " << epsilon_m << "m = " << epsilon_pix << " pixels" << std::endl;
    std::cout << "  Dilation radius: " << dilate_radius_m << "m = " << dilate_pix << " pixels" << std::endl;

    // Load map
    cv::Mat map_img = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    if (map_img.empty()) {
        std::cerr << "Failed to load map: " << map_path << std::endl;
        return 1;
    }
    std::cout << "Loaded map: " << map_path << " (" << map_img.cols << "x" << map_img.rows << ")" << std::endl;

    // === STEP 1: Binary threshold ===
    // In ROS PGM maps: 0=occupied (obstacle), 254=free, 205=unknown
    // We want obstacles (low values) to become WHITE (255)
    cv::Mat bin_img;
    cv::threshold(map_img, bin_img, 50, 255, cv::THRESH_BINARY_INV);
    // Now: obstacles are WHITE (255), free space is BLACK (0)

    // === STEP 2: Dilate BEFORE findContours ===
    // This ensures approxPolyDP simplification still covers original obstacles
    cv::Mat dilated;
    cv::dilate(bin_img, dilated, cv::getStructuringElement(cv::MORPH_RECT, 
        cv::Size(2*dilate_pix+1, 2*dilate_pix+1)));

    // === STEP 3: findContours with RETR_LIST ===
    // RETR_EXTERNAL is WRONG - it treats whole room boundary as one contour
    // RETR_LIST finds ALL contours (each wall segment separately)
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(dilated, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    std::cout << "Found " << contours.size() << " contours." << std::endl;

    // === STEP 4 & 5: approxPolyDP + Hertel-Mehlhorn ===
    std::vector<Polygon> all_polys;
    int total_edges = 0;
    
    for (const auto& contour : contours) {
        // Skip very small contours (noise)
        if (cv::contourArea(contour) < 4.0) continue;
        
        auto polys = decomposeConvex(contour, epsilon_pix);
        for (const auto& poly : polys) {
            all_polys.push_back(poly);
            total_edges += poly.size();
        }
    }
    std::cout << "Generated " << all_polys.size() << " convex polygons with " 
              << total_edges << " total edges." << std::endl;

    // === Visualize ===
    cv::Mat vis_img;
    cv::cvtColor(map_img, vis_img, cv::COLOR_GRAY2BGR);
    
    // Color palette for different polygons (like matplotlib tab20)
    std::vector<cv::Scalar> colors = {
        {255, 127, 14},  // orange
        {44, 160, 44},   // green
        {214, 39, 40},   // red
        {148, 103, 189}, // purple
        {140, 86, 75},   // brown
        {227, 119, 194}, // pink
        {127, 127, 127}, // gray
        {188, 189, 34},  // olive
        {23, 190, 207},  // cyan
        {31, 119, 180},  // blue
        {255, 187, 120}, // light orange
        {152, 223, 138}, // light green
        {255, 152, 150}, // light red
        {197, 176, 213}, // light purple
        {196, 156, 148}, // light brown
        {247, 182, 210}, // light pink
    };
    
    // Scale up first for better label visibility
    int scale = 8;
    cv::Mat vis_scaled;
    cv::resize(vis_img, vis_scaled, cv::Size(), scale, scale, cv::INTER_NEAREST);
    
    // Draw all convex polygons with different colors and labels
    for (size_t idx = 0; idx < all_polys.size(); ++idx) {
        const auto& poly = all_polys[idx];
        cv::Scalar color = colors[idx % colors.size()];
        
        // Scale polygon points and center on scaled pixels
        std::vector<cv::Point> pts;
        double cx = 0, cy = 0;
        for (const auto& p : poly) {
            // Add scale/2 offset to center polygon on the scaled pixel block
            int sx = (int)(p.x * scale) + scale / 2;
            int sy = (int)(p.y * scale) + scale / 2;
            pts.push_back(cv::Point(sx, sy));
            cx += sx;
            cy += sy;
        }
        cx /= poly.size();
        cy /= poly.size();
        
        if (pts.size() >= 3) {
            // Draw polygon outline
            cv::polylines(vis_scaled, pts, true, color, 2, cv::LINE_AA);
            
            // Draw label at centroid
            std::string label = std::to_string(idx);
            int baseline;
            cv::Size textSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.3, 1, &baseline);
            cv::Point textPos((int)cx - textSize.width/2, (int)cy + textSize.height/2);
            cv::putText(vis_scaled, label, textPos, cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1, cv::LINE_AA);
        }
    }

    // Save output
    cv::imwrite(output_path, vis_scaled);
    std::cout << "Saved visualization to: " << output_path 
              << " (scaled " << scale << "x to " << vis_scaled.cols << "x" << vis_scaled.rows << ")" << std::endl;

    return 0;
}
