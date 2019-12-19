#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <set>
#include <algorithm>
#include <map>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"
#include "Matrix4.h"

using namespace tinyxml2;
using namespace std;


// all transformation matrices required

Matrix4 Scene::cameraTransformation(Camera *c){
	Matrix4 camT;
	Matrix4 trc;
	Vec3 uuu,vvv,www;
	uuu=c->u;
	vvv=c->v;
	www=c->w;
	uuu = normalizeVec3(uuu);
	vvv= normalizeVec3(vvv);
	www= normalizeVec3(www);

	camT.val[0][0] = uuu.x; camT.val[0][1]= uuu.y; camT.val[0][2]=uuu.z;
	camT.val[1][0] = vvv.x; camT.val[1][1]= vvv.y; camT.val[1][2]=vvv.z;
	camT.val[2][0] = www.x; camT.val[2][1]= www.y; camT.val[2][2]=www.z;
	camT.val[3][3] = 1;

	trc.val[0][0] = 1; trc.val[0][3]=-c->pos.x;
	trc.val[1][1] = 1; trc.val[1][3]=-c->pos.y;
	trc.val[2][2] = 1; trc.val[2][3]=-c->pos.z;
	trc.val[3][3]= 1;

	camT = multiplyMatrixWithMatrix(camT,trc);
	return camT;
}

Matrix4 Scene::orthographicProjection(Camera* c){
	Matrix4 orthoP;
	orthoP.val[0][0] = 2/(c->right-c->left); orthoP.val[0][3]= -1*(c->right+c->left)/(c->right-c->left);
	orthoP.val[1][1] = 2/(c->top-c->bottom); orthoP.val[1][3]= -1*(c->top+c->bottom)/(c->top-c->bottom);
	orthoP.val[2][2] = -2/(c->far-c->near); orthoP.val[2][3]= -1*(c->far+c->near)/(c->far-c->near);
	orthoP.val[3][3] = 1;
	return orthoP;
}

Matrix4 Scene::perspectiveProjection(Camera* c){
	Matrix4 perP;
	perP.val[0][0] = c->near;
	perP.val[1][1] = c->near;
	perP.val[2][2] = c->far+c->near; perP.val[2][3] = c->far*c->near;
	perP.val[3][2] = -1;
	Matrix4 ort = orthographicProjection(c);
	Matrix4 res = multiplyMatrixWithMatrix(ort,perP);
	return res;
}

Matrix4 Scene::rMatrix(Rotation *r){
	Vec3 u;
	Vec3 v;
	Vec3 w;
	double pi = 3.14159265;
	u.x = r->ux; u.y = r->uy; u.z = r->uz;
	Matrix4 result;
	//u= normalizeVec3(u);
	if (min({u.x,u.y,u.z}) == u.z) {v.x = -u.y; v.y= u.x; v.z=0;}
	else if (min({u.x,u.y,u.z}) == u.y) {v.x = -u.z; v.z= u.x; v.y=0;}
	else if (min({u.x,u.y,u.z}) == u.x) {v.y = -u.z; v.z =u.y ; v.x =0;}
	v = normalizeVec3(v);
	w = crossProductVec3(u,v); w=normalizeVec3(w);
	u=normalizeVec3(u);
	Matrix4 m;
	m.val[0][0]= u.x; m.val[0][1] = u.y; m.val[0][2] = u.z;
	m.val[1][0] = v.x; m.val[1][1] = v.y; m.val[1][2] = v.z;
	m.val[2][0] = w.x; m.val[2][1] = w.y; m.val[2][2] = w.z; m.val[3][3] = 1;
	Matrix4 ro;
	ro.val[0][0]=1;
	ro.val[1][1]=cos(r->angle*pi/180); ro.val[1][2]= -1*sin(r->angle*pi/180);
	ro.val[2][1]=sin(r->angle*pi/180);
	ro.val[2][2]=cos(r->angle*pi/180);
	ro.val[3][3]=1;
	result = multiplyMatrixWithMatrix(ro,m);
	Matrix4 invM;
	invM.val[0][0]= u.x; invM.val[0][1] = v.x; invM.val[0][2] = w.x;
	invM.val[1][0] = u.y; invM.val[1][1] = v.y; invM.val[1][2] = w.y;
	invM.val[2][0] = u.z; invM.val[2][1] = v.z; invM.val[2][2] = w.z; invM.val[3][3] = 1;
	result = multiplyMatrixWithMatrix(invM,result);

	return result;

}

Matrix4 Scene::tMatrix( Translation *t){
	Matrix4 result;
	result.val[0][0] = 1; result.val[0][3] = t->tx;
	result.val[1][1] = 1; result.val[1][3] = t->ty;
	result.val[2][2] = 1; result.val[2][3] = t->tz;
	result.val[3][3] = 1;
	return result;
}

Matrix4 Scene::sMatrix( Scaling *s){
	Matrix4 result;
	result.val[0][0] = s->sx;
	result.val[1][1] = s->sy;
	result.val[2][2] = s->sz;
	result.val[3][3] =1;
	return result;
}

Matrix4 Scene::viewport(int nx , int ny){
  Matrix4 result;
	result.val[0][0] = nx/2.0; result.val[0][3] = (nx-1)/2.0;
	result.val[1][1] = ny/2.0; result.val[1][3] = (ny-1)/2.0;
	result.val[2][2] = 0.5;  result.val[2][3] = 0.5;
	result.val[3][3] = 1;
	return result;
}

void Scene::perspectiveDivide(Vec4 &p){
	if (p.t == 1 || p.t==0) return;

	p.x /= p.t;
	p.y /= p.t;
	p.z /= p.t;
	p.t /= p.t;
	return;
}

/* Backface culling */

Vec3 Scene::computeRay(Triangle tri,Camera* c,Matrix4 modmat){
	Vec3 result;
	Vec3 edge;
	Vec4 temp;
	edge = *vertices[(tri.getFirstVertexId())-1];
	temp.x= edge.x; temp.y = edge.y; temp.z=edge.z; temp.t=1;
	temp= multiplyMatrixWithVec4(modmat,temp);
	if (projectionType==1) perspectiveDivide(temp);

	edge.x=temp.x; edge.y=temp.y; edge.z=temp.z;

	//result = subtractVec3(edge,c->pos);
	result =edge;
	normalizeVec3(result);
	return result;
}

Vec3 Scene::computeNormal(Triangle tri,Matrix4 modmat){
	Vec3 a,b,c;
	Vec3 normal;
	Vec4 q,w,e;
	//b-a x c-a gives normal CCW
	a= *vertices[tri.getFirstVertexId()-1];
	b= *vertices[tri.getSecondVertexId()-1];
	c= *vertices[tri.getThirdVertexId()-1];
	q.x=a.x; q.y=a.y; q.z=a.z; q.t=1;
	w.x=b.x; w.y=b.y; w.z=b.z; w.t=1;
	e.x=c.x; e.y=c.y; e.z=c.z; e.t=1;
	q= multiplyMatrixWithVec4(modmat,q);
	w= multiplyMatrixWithVec4(modmat,w);
	e= multiplyMatrixWithVec4(modmat,e);
	if (projectionType==1){
		q.x/=q.t; q.y/=q.t; q.z/=q.t;
		w.x/=w.t; w.y/=w.t; w.z/=w.t;
		e.x/=e.t; e.y/=e.t; e.z/=e.t;}

	a.x=q.x; a.y=q.y; a.z=q.z;
	b.x=w.x; b.y=w.y; b.z=w.z;
	c.x=e.x; c.y=e.y; c.z=e.z;
	normal = crossProductVec3(subtractVec3(b,a),subtractVec3(c,a));
	normal = normalizeVec3(normal);
	return normal;
}

bool Scene::backfaceCulling(Triangle tri, Camera *cam,Matrix4 modmat){
	bool end = false;
	Vec3 ray = computeRay(tri,cam,modmat);
	Vec3 normal = computeNormal(tri,modmat);
	if (dotProductVec3(ray,normal) <0 ) {
			//ignore that triangle
			end = true;
	}
	return end;
}

Matrix4 Scene::modelMatrix(Model *model){
	Matrix4 iden = getIdentityMatrix();
	for (int i=0 ; i< model->numberOfTransformations ; i++){
		if (model->transformationTypes[i]=='r'){
			Rotation* rot;
			rot = rotations[model->transformationIds[i]-1];
			iden = multiplyMatrixWithMatrix(rMatrix(rot),iden);
		}

		else if (model->transformationTypes[i]=='s'){
			Scaling* sca;
			sca = scalings[model->transformationIds[i]-1];
			iden = multiplyMatrixWithMatrix(sMatrix(sca),iden);
		}

		 else if(model->transformationTypes[i] == 't'){
			Translation * tr;
			tr = translations[model->transformationIds[i]-1];
			iden = multiplyMatrixWithMatrix(tMatrix(tr),iden);
		}
	}
	return iden;
}

void Scene::draw(int i,int j, Color c,Camera* cam){
	if (i<0 || j<0) return;
	if (i>=cam->horRes || j>=cam->verRes) return;
	c.r = makeBetweenZeroAnd255(c.r);
	c.g = makeBetweenZeroAnd255(c.g);
	c.b = makeBetweenZeroAnd255(c.b);
	this->image[i][j].r = c.r;
	this->image[i][j].g = c.g;
	this->image[i][j].b = c.b;
 }

 
 /* checks if pixel is inside the triangle */

void Scene::triangleRasterization(double x0, double y0 , double x1, double y1, double x2, double y2,int c0,int c1,int c2,Camera* cam){
	double VeR = cam->verRes;
	double HoR = cam->horRes;
	int ymin = min({VeR,y0,y1,y2});
	int ymax = max({0.0,y0,y1,y2});
	int xmin = min({HoR,x0,x1,x2});
	int xmax = max({0.0,x0,x1,x2});
	//cout << x0<<","<<x1<<","<<x2<<".."<<xmin<<endl;
	for (int j=ymin ; j<=ymax ; j++){
		for (int i=xmin; i<=xmax ; i++){
		//for (int j=0; j<cam->verRes ;j++){
			//for (int i=0 ; i<cam->horRes ; i++){
			double alpha = ((i+0.5)*(y1-y2)+(j+0.5)*(x2-x1)+x1*y2-y1*x2 )/ (x0*(y1-y2)+y0*(x2-x1)+x1*y2-y1*x2);
			double beta = ((i+0.5)*(y2-y0)+(j+0.5)*(x0-x2)+x2*y0-y2*x0) / (x1*(y2-y0)+y1*(x0-x2)+x2*y0-y2*x0);
			double gama = ((i+0.5)*(y0-y1)+(j+0.5)*(x1-x0)+x0*y1-y0*x1) / (x2*(y0-y1)+y2*(x1-x0)+x0*y1-y0*x1);

			if (alpha>=0 && beta>=0 && gama>=0){
				Color c;
				c.r = alpha*(colorsOfVertices[c0-1]->r)+beta*(colorsOfVertices[c1-1]->r)+gama*(colorsOfVertices[c2-1]->r);
				c.g = alpha*colorsOfVertices[c0-1]->g+beta*colorsOfVertices[c1-1]->g+gama*colorsOfVertices[c2-1]->g;
				c.b = alpha*colorsOfVertices[c0-1]->b+beta*colorsOfVertices[c1-1]->b+gama*colorsOfVertices[c2-1]->b;
				draw(i,j,c,cam);
			}
		}
	}
}
void Scene::draw2(int i, int j, int c1,int c2, Camera* cam,double x0,double x1){
	if (i<0 || j<0) return;
	if (i>=cam->horRes || j>=cam->verRes) return;
	double alpha = abs((i-x0)/(x1-x0));
	Color c;
	c.r = (alpha)*(colorsOfVertices[c1-1]->r)+ (1-alpha)*(colorsOfVertices[c2-1]->r);
	c.g = (alpha)*(colorsOfVertices[c1-1]->g)+ (1-alpha)*(colorsOfVertices[c2-1]->g);
	c.b = (alpha)*(colorsOfVertices[c1-1]->b)+ (1-alpha)*(colorsOfVertices[c2-1]->b);
	c.r = makeBetweenZeroAnd255(c.r);
	c.g = makeBetweenZeroAnd255(c.g);
	c.b = makeBetweenZeroAnd255(c.b);
	this->image[i][j].r = c.r;
	this->image[i][j].g = c.g;
	this->image[i][j].b = c.b;
}

/*midpoint algorithm*/
void Scene::midpoint(double x1, double y1 , double x0, double y0,int c1,int c0,Camera* cam){

	if (abs(y1 - y0) < abs(x1 - x0)){
    if (x0 > x1)
      midlow(x1, y1, x0, y0 ,c0,c1,cam);
    else
      midlow(x0, y0, x1, y1,c0,c1,cam);
  }
  else{
    if (y0 > y1)
      midhigh(x1, y1, x0, y0,c0,c1,cam);
    else
      midhigh(x0, y0, x1, y1,c0,c1,cam);
  }
}

void Scene::midlow(double x0, double y0, double x1,double y1 ,int c0,int c1,Camera *cam){
	double dx = x1 - x0;
  double dy = y1 - y0;
  int y_i = 1;
	int boom = 0;
	if (dy < 0){
    y_i = -1;
    dy = -dy;
		boom=1;
  }
  double den = 2*dy - dx;
  int y = y0;

  for (int x=x0 ; x<=x1 ;x++){
    if (boom==1)draw2(x,y,c0,c1,cam,x0,x1);
		else draw2(x,y,c0,c1,cam,x0,x1);
		if (den > 0){
       y = y + y_i;
     	den = den - 2*dx;
    }
    den = den + 2*dy;
	}
}

void Scene::midhigh(double x0, double y0, double x1,double y1 ,int c0,int c1,Camera *cam){
	double dx = x1 - x0;
  double dy = y1 - y0;
  int x_i = 1;
	int boom=0;
  if (dx < 0){
    x_i = -1;
    dx = -dx;
		boom=1;
  }
  double den = 2*dx - dy;
  int x = x0;

  for (int y=y0 ; y<=y1 ; y++){
    if (boom==1)draw2(x,y,c1,c0,cam,x0,x1);
		else draw2(x,y,c0,c1,cam,x0,x1);
    if (den > 0){
       x = x + x_i;
     	den = den - 2*dy;
    }
    den = den + 2*dx;
	}
}

void Scene::clipping(double x0, double y0 , double x1, double y1, double x2, double y2,int c0,int c1,int c2,Camera* cam){
	midpoint(x1,y1,x0,y0,c1,c0,cam);
	midpoint(x2,y2,x0,y0,c2,c0,cam);
	midpoint(x2,y2,x1,y1,c2,c1,cam);
}



/*
	Transformations, clipping, culling, rasterization are done here.

*/
void Scene::forwardRenderingPipeline(Camera *camera)
{

	Matrix4 master;
	if (projectionType ==0){//orthog

		master = multiplyMatrixWithMatrix(orthographicProjection(camera),cameraTransformation(camera));
	}
	else if (projectionType ==1){ //perspective
		master = multiplyMatrixWithMatrix(perspectiveProjection(camera),cameraTransformation(camera));
	}

	Matrix4 vp = viewport(camera->horRes,camera->verRes);
	//Matrix4 temp = cameraTransformation(camera);
	int msize = this->models.size();
	for (int i = 0; i<msize ; i++){
		Model* curModel = models[i];
		Matrix4 neo_model = modelMatrix(curModel);
		Matrix4 master2 = multiplyMatrixWithMatrix(master,neo_model);
		//Matrix4 tryhard = multiplyMatrixWithMatrix(master,neo_model);
		//tryhard = multiplyMatrixWithMatrix(vp,tryhard);
		set<int> v_index;
		vector<int> oneorzero;

		//temp = multiplyMatrixWithMatrix(temp,neo_model);
		for (int j=0; j< curModel->numberOfTriangles ; j++){
			Triangle tri = curModel->triangles[j];
			if (cullingEnabled==true){
				if (backfaceCulling(tri,camera,master2)==true){
					oneorzero.push_back(0);
					v_index.insert(tri.getFirstVertexId()); v_index.insert(tri.getSecondVertexId());v_index.insert(tri.getThirdVertexId());
				}
				else{
				oneorzero.push_back(1);
				v_index.insert(tri.getFirstVertexId()); v_index.insert(tri.getSecondVertexId());v_index.insert(tri.getThirdVertexId());
				}
			}
			else{
			oneorzero.push_back(1);
			v_index.insert(tri.getFirstVertexId()); v_index.insert(tri.getSecondVertexId());v_index.insert(tri.getThirdVertexId());}
		}

		map<int,Vec4> cpVertex;
		for (set<int>::iterator it = v_index.begin(); it!= v_index.end();it++){
			Vec4 point(vertices[*it-1]->x,vertices[*it-1]->y,vertices[*it-1]->z,1.0,*it);
			//cout << point.x <<"-" << point.y <<"-" << point.z <<"-" << point.t << endl;
			Vec4 p_times_everything = multiplyMatrixWithVec4(master2,point);
			//cout << p_times_everything.x <<"-" <<p_times_everything.y <<"-" << p_times_everything.z << "-" <<p_times_everything.t << endl;
		 /*check*/
				if (projectionType==1 ) perspectiveDivide(p_times_everything);
				p_times_everything = multiplyMatrixWithVec4(vp,p_times_everything);
				cpVertex[*it-1] = p_times_everything;
		}
		


		if (curModel->type ==0) { //wireframe clip it
					int index=0;
					for (vector<Triangle>::iterator curTri = curModel->triangles.begin(); curTri != curModel->triangles.end();curTri++){

						if (oneorzero[index] ==1){
							double x_0 = cpVertex[curTri->getFirstVertexId()-1].x;
							double y_0 = cpVertex[curTri->getFirstVertexId()-1].y;
							double x_1 = cpVertex[curTri->getSecondVertexId()-1].x;
							double y_1 = cpVertex[curTri->getSecondVertexId()-1].y;
							double x_2 = cpVertex[curTri->getThirdVertexId()-1].x;
							double y_2 = cpVertex[curTri->getThirdVertexId()-1].y;
							int cid1,cid2,cid3;
							//cout << x_0 <<"zero " << x_1 <<" one "<< endl;
							cid1= cpVertex[curTri->getFirstVertexId()-1].colorId;
							cid2= cpVertex[curTri->getSecondVertexId()-1].colorId;
							cid3= cpVertex[curTri->getThirdVertexId()-1].colorId;
							clipping(x_0,y_0,x_1,y_1,x_2,y_2,cid1,cid2,cid3,camera);
						}
						index++;

					}
			}
			else if (curModel->type ==1){ // solid
				int index=0;
				for (vector<Triangle>::iterator curTri = curModel->triangles.begin(); curTri != curModel->triangles.end();curTri++){

					if (oneorzero[index]==1){
					double x_0 = cpVertex[curTri->getFirstVertexId()-1].x;
					double y_0 = cpVertex[curTri->getFirstVertexId()-1].y;
					double x_1 = cpVertex[curTri->getSecondVertexId()-1].x;
					double y_1 = cpVertex[curTri->getSecondVertexId()-1].y;
					double x_2 = cpVertex[curTri->getThirdVertexId()-1].x;
					double y_2 = cpVertex[curTri->getThirdVertexId()-1].y;
					int cid1,cid2,cid3;
					cid1= cpVertex[curTri->getFirstVertexId()-1].colorId;
					cid2= cpVertex[curTri->getSecondVertexId()-1].colorId;
					cid3= cpVertex[curTri->getThirdVertexId()-1].colorId;
					//cout << x_0<<","<<y_0<< ","<< x_1 <<"," <<y_1<<endl;
					//return;
					triangleRasterization(x_0,y_0,x_1,y_1,x_2,y_2,cid1,cid2,cid3,camera);
					}
					index++;
				}
			}
	}
}







/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL)
		pElement->QueryBoolText(&cullingEnabled);

	// read projection type
	pElement = pRoot->FirstChildElement("ProjectionType");
	if (pElement != NULL)
		pElement->QueryIntText(&projectionType);

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read models
	pElement = pRoot->FirstChildElement("Models");

	XMLElement *pModel = pElement->FirstChildElement("Model");
	XMLElement *modelElement;
	while (pModel != NULL)
	{
		Model *model = new Model();

		pModel->QueryIntAttribute("id", &model->modelId);
		pModel->QueryIntAttribute("type", &model->type);

		// read model transformations
		XMLElement *pTransformations = pModel->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		pTransformations->QueryIntAttribute("count", &model->numberOfTransformations);

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			model->transformationTypes.push_back(transformationType);
			model->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		// read model triangles
		XMLElement *pTriangles = pModel->FirstChildElement("Triangles");
		XMLElement *pTriangle = pTriangles->FirstChildElement("Triangle");

		pTriangles->QueryIntAttribute("count", &model->numberOfTriangles);

		while (pTriangle != NULL)
		{
			int v1, v2, v3;

			str = pTriangle->GetText();
			sscanf(str, "%d %d %d", &v1, &v2, &v3);

			model->triangles.push_back(Triangle(v1, v2, v3));

			pTriangle = pTriangle->NextSiblingElement("Triangle");
		}

		models.push_back(model);

		pModel = pModel->NextSiblingElement("Model");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	// if image is filled before, just change color rgb values with the background color
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}
