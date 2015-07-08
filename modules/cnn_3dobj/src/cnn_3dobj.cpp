#include "precomp.hpp"
using namespace cv;
using namespace std;

namespace cv{ namespace cnn_3dobj{

	IcoSphere::IcoSphere(float radius_in, int depth_in)
	{

		X = 0.525731112119133606f;
		Z = 0.850650808352039932f;
		radius = radius_in;
		depth = depth_in;
		X *= radius;
		Z *= radius;
		cout << "depth " << depth << endl;
		double vdata[12][3] = { { -X, 0.0f, Z }, { X, 0.0f, Z },
				{ -X, 0.0f, -Z }, { X, 0.0f, -Z }, { 0.0f, Z, X }, { 0.0f, Z, -X },
				{ 0.0f, -Z, X }, { 0.0f, -Z, -X }, { Z, X, 0.0f }, { -Z, X, 0.0f },
				{ Z, -X, 0.0f }, { -Z, -X, 0.0f } };


		int tindices[20][3] = { { 0, 4, 1 }, { 0, 9, 4 }, { 9, 5, 4 },
				{ 4, 5, 8 }, { 4, 8, 1 }, { 8, 10, 1 }, { 8, 3, 10 }, { 5, 3, 8 },
				{ 5, 2, 3 }, { 2, 7, 3 }, { 7, 10, 3 }, { 7, 6, 10 }, { 7, 11, 6 },
				{ 11, 0, 6 }, { 0, 1, 6 }, { 6, 1, 10 }, { 9, 0, 11 },
				{ 9, 11, 2 }, { 9, 2, 5 }, { 7, 2, 11 } };

		std::vector<double>* texCoordsList = new std::vector<double>;
		std::vector<int>* indicesList = new std::vector<int>;

		// Iterate over points
		for (int i = 0; i < 20; ++i) {

			subdivide(vdata[tindices[i][0]], vdata[tindices[i][1]],
					vdata[tindices[i][2]], depth);
		}

		set<vector<double> > set_t;
		for (int j = 0; j<int(CameraPos.size()); j++){
		  vector<double> tmp(3);
		  tmp[0] = CameraPos.at(j).x;
		  tmp[1] = CameraPos.at(j).y;
		  tmp[2] = CameraPos.at(j).z;
		  set_t.insert(tmp);
		}


		/*CameraPos_temp.push_back(CameraPos[0]);
		for (int j = 1; j<int(CameraPos.size()); j++)
			{
				for (int k = 0; k<int(CameraPos_temp.size()); k++)
				{
					if (CameraPos.at(j) == CameraPos_temp.at(k))
						break;
					CameraPos_temp.push_back(CameraPos[j]);
				}
			}
		CameraPos = CameraPos_temp;*/
		/*CameraPos.erase(unique(CameraPos.begin(), CameraPos.end()), CameraPos.end());

		for (int j = 1; j<int(CameraPos.size()); j++){
		  cout << CameraPos.at(j) << endl;
		}
		cout << endl;*/

		CameraPos.clear();
		for (set<vector<double> >::iterator j = set_t.begin(); j != set_t.end(); j++){
		  CameraPos.push_back(Point3d(j->at(0), j->at(1), j->at(2)));
    }

		cout << "View points in total: " << CameraPos.size() << endl;
		cout << "The coordinate of view point: " << endl;
		for(int i=0; i < (int)CameraPos.size(); i++) {
			//cout << CameraPos.at(i).x <<' '<< CameraPos.at(i).y << ' ' << CameraPos.at(i).z << endl;
		}
	}


	void IcoSphere::norm(double v[])
	{

	  double len = 0;

		for (int i = 0; i < 3; ++i) {
			len += v[i] * v[i];
		}

		len = sqrt(len);

		for (int i = 0; i < 3; ++i) {
			v[i] /= ((double)len)/((double)radius);
		}
	}

	void IcoSphere::add(double v[])
	{
		Point3f temp_Campos;
		std::vector<double>* temp = new std::vector<double>;
		for (int k = 0; k < 3; ++k) {
			vertexList.push_back(v[k]);
			vertexNormalsList.push_back(v[k]);
			temp->push_back(v[k]);
		}
		temp_Campos.x = temp->at(0);temp_Campos.y = temp->at(1);temp_Campos.z = temp->at(2);
		CameraPos.push_back(temp_Campos);
	}

	void IcoSphere::subdivide(double v1[], double v2[], double v3[], int depth)
	{

		norm(v1);
		norm(v2);
		norm(v3);
		if (depth == 0) {
			add(v1);
			add(v2);
			add(v3);
			return;
		}

		double* v12 = new double[3];
		double* v23 = new double[3];
		double* v31 = new double[3];

		for (int i = 0; i < 3; ++i) {
			v12[i] = (v1[i] + v2[i]) / 2;
			v23[i] = (v2[i] + v3[i]) / 2;
			v31[i] = (v3[i] + v1[i]) / 2;
		}

		norm(v12);
		norm(v23);
		norm(v31);

		subdivide(v1, v12, v31, depth - 1);
		subdivide(v2, v23, v12, depth - 1);
		subdivide(v3, v31, v23, depth - 1);
		subdivide(v12, v23, v31, depth - 1);
	}
}}
