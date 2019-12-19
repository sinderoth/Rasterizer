#ifndef _SCENE_H_
#define _SCENE_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "Vec4.h"
#include "Matrix4.h"

using namespace std;

class Scene
{
public:
	Color backgroundColor;
	bool cullingEnabled;
	int projectionType;

	vector< vector<Color> > image;
	vector< Camera* > cameras;
	vector< Vec3* > vertices;
	vector< Color* > colorsOfVertices;
	vector< Scaling* > scalings;
	vector< Rotation* > rotations;
	vector< Translation* > translations;
	vector< Model* > models;

	Scene(const char *xmlPath);

	void initializeImage(Camera* camera);
	void forwardRenderingPipeline(Camera* camera);
	int makeBetweenZeroAnd255(double value);
	void writeImageToPPMFile(Camera* camera);
	void convertPPMToPNG(string ppmFileName, int osType);

	Matrix4 cameraTransformation(Camera* c);
	Matrix4 orthographicProjection(Camera* c);
	Matrix4 perspectiveProjection(Camera *c);

	Matrix4 rMatrix(Rotation * r);
	Matrix4 tMatrix(Translation * t);
	Matrix4 sMatrix(Scaling * s);

	Matrix4 viewport(int nx , int ny);
	Matrix4 modelMatrix(Model * model);

	void perspectiveDivide(Vec4 &p);
	Vec3 computeRay(Triangle tri,Camera* c,Matrix4 m);
	Vec3 computeNormal(Triangle tri,Matrix4 m);

	bool backfaceCulling(Triangle tri , Camera *cam,Matrix4 m);

	void triangleRasterization(double x0, double y0 , double x1, double y1, double x2, double y2,int c1,int c2,int c3,Camera* cam);
	void draw(int i,int j, Color c,Camera* cam);


	void draw2(int i, int j, int c1,int c2, Camera* cam,double x0,double x1);
	void midpoint(double x1, double y1 , double x0, double y0,int c1,int c0,Camera* cam);
	void clipping(double x0, double y0 , double x1, double y1, double x2, double y2,int c0,int c1,int c2,Camera* cam);

	void midlow(double x0, double y0, double x1,double y1 ,int c0,int c1,Camera *cam);
	void midhigh(double x0, double y0, double x1,double y1 ,int c0,int c1,Camera *cam);
};

#endif
