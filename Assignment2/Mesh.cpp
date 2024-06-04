#include "Mesh.h"
#include <tuple>
#include <string>
#define MAX_BUFFER_SIZE 128

using namespace std;

void Mesh::load( const char* filename )
{
	// 2.1.1. load() should populate bindVertices, currentVertices, and faces

	ifstream input;
	input.open(filename);

	if (!input.is_open()) {
		throw std::runtime_error("Could not open file");
	}

	string line;
	while (getline(input, line)) {
		stringstream ss(line);
		string s;
		Tuple3u face;
		Vector3f vertex;

		ss >> s;

		if (s == "v") {
			ss >> vertex[0] >> vertex[1] >> vertex[2];
			bindVertices.push_back(vertex);
		} else if (s == "f") {
			ss >> face[0] >> face[1] >> face[2];
			faces.push_back(face);
		}

	}

	input.close();

	// make a copy of the bind vertices as the current vertices
	currentVertices = bindVertices;
}

void Mesh::draw()
{
	// Since these meshes don't have normals
	// be sure to generate a normal per triangle.
	// Notice that since we have per-triangle normals
	// rather than the analytical normals from
	// assignment 1, the appearance is "faceted".

	int a, b, c;

	glBegin(GL_TRIANGLES);
	for (unsigned i = 0; i < faces.size(); i++) {
		a = faces[i][0] - 1;
		b = faces[i][1] - 1;
		c = faces[i][2] - 1;

		Vector3f norm = Vector3f::cross(currentVertices[b] - currentVertices[a], currentVertices[c] - currentVertices[a]).normalized();

		glNormal3d(norm[0], norm[1], norm[2]);
		glVertex3d(currentVertices[a][0], currentVertices[a][1], currentVertices[a][2]);
		glVertex3d(currentVertices[b][0], currentVertices[b][1], currentVertices[b][2]);
		glVertex3d(currentVertices[c][0], currentVertices[c][1], currentVertices[c][2]);
	}
	glEnd();
}

void Mesh::loadAttachments( const char* filename, int numJoints )
{
	// 2.2. Implement this method to load the per-vertex attachment weights
	// this method should update m_mesh.attachments
	ifstream input;
	input.open(filename);

	if (!input.is_open()) {
		throw std::runtime_error("Could not open file");
	}

	string line;
	while (getline(input, line)) {
		vector<float> weights;
		stringstream ss(line);

		for (int i = 0; i < numJoints - 1; i++) {
			float weight;
			ss >> weight;
			weights.push_back(weight);
		}

		attachments.push_back(weights);
	}

	input.close();
}
