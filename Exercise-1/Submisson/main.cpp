#include <iostream>
#include <fstream>
#include <array>

#include "Eigen.h"
#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two   per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Determine number of valid faces
	// get by for loop and condition filters invalid vertices and too far legnth
	unsigned nFaces = 0; 

	std::ostringstream oss;
	// Last row and column should not participate the grid triangulation
	for (int idx_row = 0; idx_row < height - 1; idx_row++) {
		for (int idx_col = 0; idx_col < width - 1; idx_col++) {

			// index of participating point (idx_row, idx_col) => major row array
			int basePoint_idx = idx_row * width + idx_col; // This point
			int rightPoint_idx = basePoint_idx + 1; // Right point
			int bottomPoint_idx = basePoint_idx + width; // Bottom point
			int bottomRightPoint_idx = bottomPoint_idx + 1; // Bottom Right point

			Vector4f basePointVertex = vertices[basePoint_idx].position;
			Vector4f rightPointVertex = vertices[rightPoint_idx].position;
			Vector4f bottomPointVertex = vertices[bottomPoint_idx].position;
			Vector4f bottomRightPointVertex = vertices[bottomRightPoint_idx].position;

			// test the vertices are valid? => Norm shouldn't be INFINITY
			bool basePointVertex_isValid = vertices[basePoint_idx].position.norm() != INFINITY;
			bool rightPointVertex_isValid = vertices[rightPoint_idx].position.norm() != INFINITY;
			bool bottomPointVertex_isValid = vertices[bottomPoint_idx].position.norm() != INFINITY;
			bool bottomRightPointVertex_isValid = vertices[bottomRightPoint_idx].position.norm() != INFINITY;

			// test edge length should smaller than threshold
			bool dis_Base_Right = (basePointVertex - rightPointVertex).norm() < edgeThreshold;
			bool dis_Base_Bottom = (basePointVertex - bottomPointVertex).norm() < edgeThreshold;
			bool dis_Bottom_BottomRight = (bottomPointVertex - bottomRightPointVertex).norm() < edgeThreshold;
			bool dis_Right_BottomRight = (rightPointVertex - bottomRightPointVertex).norm() < edgeThreshold;

			// upper triangle 
			if (basePointVertex_isValid && rightPointVertex_isValid && bottomPointVertex_isValid && dis_Base_Right && dis_Base_Bottom) {
				oss << "3 " << basePoint_idx << " " << bottomPoint_idx << " " << rightPoint_idx << std::endl;
				nFaces++;
			}

			// downer triangle
			if (rightPointVertex_isValid && bottomPointVertex_isValid && bottomRightPointVertex_isValid && dis_Bottom_BottomRight && dis_Right_BottomRight) {
				oss << "3 " << bottomPoint_idx << " " << bottomRightPoint_idx << " " << rightPoint_idx << std::endl;
				nFaces++;
			}
		}
	}

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;

	outFile << "# numVertices numFaces numEdges" << std::endl;

	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	outFile << "# list of vertices" << std::endl;
	outFile << "# X Y Z R G B A" << std::endl;

	// TODO: save vertices
	for (int idx_vertex = 0; idx_vertex < nVertices; idx_vertex++) {
		if (vertices[idx_vertex].position.x() == MINF) {
			outFile << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << std::endl;
		}
		else {
			outFile << vertices[idx_vertex].position.x() << " " << vertices[idx_vertex].position.y() << " " << vertices[idx_vertex].position.z() << " " << (unsigned int)vertices[idx_vertex].color[0] << " " << (unsigned int)vertices[idx_vertex].color[1] << " " << (unsigned int)vertices[idx_vertex].color[2] << " " << (unsigned int)vertices[idx_vertex].color[3] << std::endl;
		}
	}


	// TODO: save valid faces
	outFile << "# list of faces" << std::endl;
	outFile << "# nVerticesPerFace idx0 idx1 idx2 ..." << std::endl;
	outFile << oss.str() << std::endl;

	// close file
	outFile.close();

	return true;
}

int main()
{
	// Make sure this path points to the data folder
	std::string filenameIn = "C:\\Users\\ge63rem\\Desktop\\TUM\\WS2122\\3D\\Execrises\\Data\\rgbd_dataset_freiburg1_xyz\\";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		Matrix3f depthIntrinsicsInv = depthIntrinsics.inverse();

		float fX = depthIntrinsics(0, 0);
		float fY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];
		
		for (int idx_row = 0; idx_row < sensor.GetDepthImageHeight(); idx_row++) {
			for (int idx_col = 0; idx_col < sensor.GetDepthImageWidth(); idx_col++) {
				// Position in array idx_row * sensor.GetDepthImageWidth() + idx_col for (idx_row, idx_col) in screen panel
				int idx = idx_row * sensor.GetDepthImageWidth() + idx_col;
				float z_depth = depthMap[idx];
				if (z_depth == MINF) {
					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);
				} else {
					float x = (idx_col - cX) / fX;
					float y = (idx_row - cY) / fY;
					
					Vector3f point_screen = Vector3f(x * z_depth, y * z_depth, z_depth);
					
					// Vector3f point_screen = Vector3f((idx_row - cX) / fX * z_depth, (idx_col - cY) / fY * z_depth, z_depth);
					Vector3f point_intrinsicsInv = point_screen;
					Vector4f point_intrinsicsInv_Homo = Vector4f(point_intrinsicsInv.x(), point_intrinsicsInv.y(), point_intrinsicsInv.z(), 1);
					Vector4f point_world = trajectoryInv * (depthExtrinsicsInv * point_intrinsicsInv_Homo);

					vertices[idx].position = point_world;
					vertices[idx].color = Vector4uc(colorMap[4 * idx], colorMap[4 * idx + 1], colorMap[4 * idx + 2], colorMap[4 * idx + 3]);
				}
			}
		}

		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}