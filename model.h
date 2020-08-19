#ifndef __MODEL_H__
#define __MODEL_H__

#include <vector>
#include "geometry.h"

class Model {
private:
	std::vector<Vec3f> verts_;
	std::vector<Vec2f> text_cords_;
	std::vector<Vec3f> norm_cords_;
	std::vector<std::vector<int> > faces_;
	std::vector<std::vector<int>> textures_;
	std::vector<std::vector<int>> normals_;
public:
	Model(const char *filename);
	~Model();
	int nverts();
	int nfaces();
	Vec3f vert(int i);
	Vec2f text(int i);
	Vec3f normal(int i);
	std::vector<int> face(int idx);
	std::vector<int> texture(int i);
	std::vector<int> norm(int idx);

};

#endif //__MODEL_H__
