#include "ArmorerPhys/os.h"

#include <string>

#include "ArmorerPhys/type.h"
#include "igl/writeOBJ.h"
#include "igl/readMESH.h"
#include "igl/writeMESH.h"
#include "igl/readDMAT.h"
#include "igl/writeDMAT.h"

namespace aphys {

void writeOBJ(const std::string& filename, const MatxXd& V, const MatxXi& F) {
  if (V.cols() == 2) {
    MatxXd V3 = V;
    V3.conservativeResize(V.rows(), 3);
    V3.col(2).setZero();
    igl::writeOBJ(filename, V3, F);
  } else if (V.cols() == 3) {
    igl::writeOBJ(filename, V, F);
  } else {
    throw std::runtime_error("writeOBJ: V must have 2 or 3 columns");
  }
}

void export2DPolygonToSVG(const MatxXd& verts, const Matx2i& edge,
                          const Matx3i& faces, const Matx3f& colors,
                          const RGB& color, const std::string& filename,
                          double scale) {
  std::ofstream svgFile(filename);
  if (!svgFile.is_open()) {
    std::cerr << "Cannot open file: " << filename << std::endl;
    return;
  }

  MatxXd V = verts * scale;

  double minX = V.col(0).minCoeff();
  double maxX = V.col(0).maxCoeff();
  double minY = V.col(1).minCoeff();
  double maxY = V.col(1).maxCoeff();

  double width = maxX - minX;
  double height = maxY - minY;

  // Add margins
  double margin = width * 0.1;
  minX -= margin;
  minY -= margin;
  width += 2 * margin;
  height += 2 * margin;

  double per_face_color_mode = colors.rows() > 0 ? true : false;

  // Write SVG header
  svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
          << "width=\"" << width << "\" height=\"" << height << "\" "
          << "viewBox=\"" << minX << " " << minY << " " << width << " "
          << height << "\">\n";

  // Iterate over each face
  for (int i = 0; i < faces.rows(); ++i) {
    // Get vertex indices
    int v0 = faces(i, 0);
    int v1 = faces(i, 1);
    int v2 = faces(i, 2);

    // Get vertex positions
    double x0 = V(v0, 0);
    double y0 = V(v0, 1);
    double x1 = V(v1, 0);
    double y1 = V(v1, 1);
    double x2 = V(v2, 0);
    double y2 = V(v2, 1);

    // Get color
    int R = color(0);
    int G = color(1);
    int B = color(2);

    if (per_face_color_mode) {
      float r = colors(i, 0);
      float g = colors(i, 1);
      float b = colors(i, 2);
      int R = static_cast<int>(r * 255);
      int G = static_cast<int>(g * 255);
      int B = static_cast<int>(b * 255);
    }

    // Convert color to hex format
    char colorHex[8];
    snprintf(colorHex, sizeof(colorHex), "#%02X%02X%02X", R, G, B);

    // Write the polygon element
    svgFile << "<polygon points=\"" << x0 << "," << y0 << " " << x1 << "," << y1
            << " " << x2 << "," << y2 << "\" "
            << "style=\"fill:" << colorHex
            << ";stroke:black;stroke-width:0.1\" />\n";
  }

  // Close the SVG element
  svgFile << "</svg>\n";

  // Close the file
  svgFile.close();
}

void exportPolygonToPointBasedSVG(const MatxXd& V, const Matx2i& edge,
                                  const Matx3i& faces, const Matx3f& colors,
                                  const RGB& color, const std::string& filename,
                                  float radius, float edge_width,
                                  double stroke_width, double scale,
                                  bool transparent) {
  // Open the SVG file
  std::ofstream svgFile(filename);
  if (!svgFile.is_open()) {
    std::cerr << "Cannot open file: " << filename << std::endl;
    return;
  }

  MatxXd verts = scale * V;

  // Determine bounding box for SVG canvas
  double minX = verts.col(0).minCoeff();
  double maxX = verts.col(0).maxCoeff();
  double minY = verts.col(1).minCoeff();
  double maxY = verts.col(1).maxCoeff();

  double width = maxX - minX;
  double height = maxY - minY;

  // Add margins
  double margin = width * 0.1;
  minX -= margin;
  minY -= margin;
  width += 2 * margin;
  height += 2 * margin;

  // Write SVG header
  svgFile << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
          << "width=\"" << width << "\" height=\"" << height << "\" "
          << "viewBox=\"" << minX << " " << minY << " " << width << " "
          << height << "\">\n";

  // If not transparent, render faces
  if (!transparent) {
    // Convert face color to hex format
    int R = color(0);
    int G = color(1);
    int B = color(2);
    char colorHex[8];
    snprintf(colorHex, sizeof(colorHex), "#%02X%02X%02X", R, G, B);

    // Iterate over each face
    for (int i = 0; i < faces.rows(); ++i) {
      // Get vertex indices
      int v0 = faces(i, 0);
      int v1 = faces(i, 1);
      int v2 = faces(i, 2);

      // Get vertex positions
      double x0 = verts(v0, 0);
      double y0 = verts(v0, 1);
      double x1 = verts(v1, 0);
      double y1 = verts(v1, 1);
      double x2 = verts(v2, 0);
      double y2 = verts(v2, 1);

      // Write the polygon element
      svgFile << "<polygon points=\"" << x0 << "," << y0 << " " << x1 << ","
              << y1 << " " << x2 << "," << y2 << "\" "
              << "style=\"fill:" << colorHex
              << ";stroke:black;stroke-width:" << stroke_width << "\" />\n";
    }
  }

  // Render edges
  for (int i = 0; i < edge.rows(); ++i) {
    int v0 = edge(i, 0);
    int v1 = edge(i, 1);

    // Get positions
    double x0 = verts(v0, 0);
    double y0 = verts(v0, 1);
    double x1 = verts(v1, 0);
    double y1 = verts(v1, 1);

    // Write the line element
    svgFile << "<line x1=\"" << x0 << "\" y1=\"" << y0 << "\" x2=\"" << x1
            << "\" y2=\"" << y1
            << "\" style=\"stroke:black;stroke-width:" +
                   std::to_string(edge_width) + "\" />\n";
  }

  // Render vertex points
  /*for (int i = 0; i < verts.rows(); ++i) {
    double x = verts(i, 0);
    double y = verts(i, 1);

    // Get color
    float r = colors(i, 0);
    float g = colors(i, 1);
    float b = colors(i, 2);

    // Convert color to hex format
    int R = static_cast<int>(r * 255);
    int G = static_cast<int>(g * 255);
    int B = static_cast<int>(b * 255);
    char colorHex[8];
    snprintf(colorHex, sizeof(colorHex), "#%02X%02X%02X", R, G, B);

    // Write circle element
    svgFile << "<circle cx=\"" << x << "\" cy=\"" << y << "\" r=\"" << radius
            << "\" "  // Use the dynamic radius
            << "fill=\"" << colorHex << "\" "
            << "stroke=\"black\" "  // Stroke color set to black
            << "stroke-width=\"" << stroke_width << "\" />\n";
  }*/

  // Close the SVG element
  svgFile << "</svg>\n";

  // Close the file
  svgFile.close();
}

}  // namespace aphys
