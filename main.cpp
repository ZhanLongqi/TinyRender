#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include <limits>
const int width  = 800;
const int height = 800;
const int depth=255;
const Vec3f eye=Vec3f(0,0,3);
const Vec3f center=Vec3f(0,0,0);
const Vec3f light_dir=Vec3f(0,0,-1);
const Vec3f look_dir=center-eye;
const Vec3f top=Vec3f(0,1,0);
const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0,255,0,255);
const TGAColor blue =TGAColor(0,0,255,255);
Model *model = NULL;
Matrix ModelView=Matrix::identity();
void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0-x1)<std::abs(y0-y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0>x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }
    int derror=std::abs(y1-y0)*2;
    int error=0;
    int y=y0;
    for (int x=x0; x<=x1; x++) {
        //float t = (x-x0)/(float)(x1-x0);
        //int y = y0*(1.-t) + y1*t;
        error+=derror;
        if(error>(x1-x0)){
            y+=(y1-y0)>0?1:-1;
            error-=2*(x1-x0);
        }
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void line(Vec2i v0,Vec2i v1,TGAImage &image,TGAColor color){
    int x0=v0.x;
    int y0=v0.y;
    int x1=v1.x;
    int y1=v1.y;
    line(x0,y0,x1,y1,image,color); 
}

Vec2i project_normal(Vec3i vet){
    return Vec2i(vet.x,vet.y);
}

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
    Vec3f s[2];
    for (int i=2; i--; ) {
        s[i][0] = C[i]-A[i];
        s[i][1] = B[i]-A[i];
        s[i][2] = A[i]-P[i];
    }
    Vec3f u = cross(s[0], s[1]);
    if (std::abs(u[2])>1e-2) // dont forget that u[2] is integer. If it is zero then triangle ABC is degenerate
        return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z);
    return Vec3f(-1,1,1); // in this case generate negative coordinates, it will be thrown away by the rasterizator
}
Matrix lookat(Vec3f eye,Vec3f center,Vec3f g,Vec3f t){
    Matrix m1=Matrix::identity();
    Vec3f gt=cross(g,t).normalize();
    for(int i=0;i<3;i++){
        m1[0][i]=gt[i];
        m1[1][i]=t[i];
        m1[2][i]=-g[i];
        m1[3][i]=center[i]-eye[i];
    }
    return m1;
}

Matrix viewport(int x,int y,int w,int h){
    Matrix m=Matrix::identity();
    m[0][3]=x+w/2.f;
    m[1][3]=y+h/2.f;
    m[2][3]=depth/2.f;

    m[0][0]=w/2.f;
    m[1][1]=h/2.f;
    m[2][2]=depth/2.f;
    return m;
}
Matrix v2m(Vec3f v){
    mat<4,4,float> m;
    m[0][0]=v.x;
    m[0][1]=v.y;
    m[0][2]=v.z;
    m[0][3]=1.f;
    return m;
}

Vec3f m2v(Matrix m){
    return Vec3f(m[0][0],m[0][1],m[0][2]);
}

Vec3f world2screen(Vec3f v) {
    std::cout<<"flag"<<std::endl;
    static Matrix ViewPort=viewport(0,0,width,height);
    static Matrix mvp=lookat(eye,Vec3f(0,0,0),look_dir,top);
    static Matrix projection=ViewPort*mvp;
    for(int i=0;i<4;i++){
        for(int j=0;j<4;j++){
            std::cout<<projection[i][j]<<" ";
        }
        std::cout<<""<<std::endl;
    }
    return m2v(projection*v2m(v));
}

void triangle(Vec3f *pts,float *zbuffer,TGAImage &image,TGAColor color){
    Vec2f bboxmin(std::numeric_limits<int>::max(),std::numeric_limits<int>::max());
    Vec2f bboxmax(-std::numeric_limits<int>::max(),-std::numeric_limits<int>::max());
    Vec2f clamp(image.get_width()-1,image.get_height()-1);
    for(int i=0;i<3;i++){
        for(int j=0;j<2;j++){
            bboxmax[j]=std::min(clamp[j],std::max(bboxmax[j],pts[i][j]));
            bboxmin[j]=std::max(0.0f,std::min(bboxmin[j],pts[i][j]));
        }
    }
    Vec3f p;
    for(p.x=bboxmin[0];p.x<bboxmax[0];p.x++){
        for(p.y=bboxmin[1];p.y<bboxmax[1];p.y++){
            Vec3f screen_bc=barycentric(pts[0],pts[1],pts[2],p);
            if(screen_bc.x<0||screen_bc.y<0||screen_bc.z<0)continue;
            p.z=0;
            for(int i=0;i<3;i++){
                p.z+=pts[i][2]*screen_bc[i];
            }
            if(p.z>zbuffer[int(p.x+p.y*width)]){
                zbuffer[int(p.x+p.y*width)]=p.z;
                image.set(p.x,p.y,color);
            }
        }
    }
    
}






void triangle_texture(Vec3f *pts,Vec2f *texture_cord,Vec3f *normal_cord,float *zbuffer,TGAImage &image,TGAImage &texture,Vec3f light_dir){
    Vec2f bboxmin(std::numeric_limits<int>::max(),std::numeric_limits<int>::max());
    Vec2f bboxmax(-std::numeric_limits<int>::max(),-std::numeric_limits<int>::max());
    Vec2f clamp(image.get_width()-1,image.get_height()-1);
    for(int i=0;i<3;i++){
        for(int j=0;j<2;j++){
            bboxmax[j]=std::min(clamp[j],std::max(bboxmax[j],pts[i][j]));
            bboxmin[j]=std::max(0.0f,std::min(bboxmin[j],pts[i][j]));
        }
    }
    Vec3f p;


    for(p.x=bboxmin[0];p.x<bboxmax[0];p.x++){
        for(p.y=bboxmin[1];p.y<bboxmax[1];p.y++){
            Vec3f bc_cord=barycentric(pts[0],pts[1],pts[2],p);
            if(bc_cord.x<0||bc_cord.y<0||bc_cord.z<0)continue;
            p.z=0;
            int x=0,y=0;
            TGAColor color;
            Vec3f normal;
            for(int i=0;i<3;i++){
                p.z+=pts[i][2]*bc_cord[i];
            }

         //   color=TGAColor(255*intensity,255*intensity,255*intensity,255);
           // std::cout<<x<<" "<<y<<std::endl;
           // std::cout<<p.y<<" "<<p.x<<std::endl;
            if(p.z>zbuffer[int(p.x+p.y*width)]){
                for(int i=0;i<3;i++){
                    x+=int((texture_cord[i][0])*bc_cord[i]*texture.get_width());
                    y+=int((texture_cord[i][1])*bc_cord[i]*texture.get_height());
                    normal=normal+normal_cord[i]*bc_cord[i];
                }
            normal.normalize();
            float intensity=normal*light_dir;
            intensity*=-1;
            //<<intensity<<std::endl;
            color=texture.get(x,y);//TGAColor(255*intensity,255*intensity,255*intensity,255);//
            //color=TGAColor(color[0]*intensity,color[1]*intensity,color[2]*intensity,255);
                zbuffer[int(p.x+p.y*width)]=p.z;
                if(intensity>0)
                image.set(p.x,p.y,color);
            }
        }
    }
}

void triangle_line(Vec2i vet0,Vec2i vet1,Vec2i vet2,TGAImage &image,TGAColor color){
    if(vet0.y>vet1.y)std::swap(vet0,vet1);
    if(vet1.y>vet2.y)std::swap(vet1,vet2);
    if(vet0.y>vet1.y)std::swap(vet0,vet1);
    
    line(vet0,vet1,image,color);
    line(vet1,vet2,image,color);
    line(vet2,vet0,image,color);

}

void rasterize(Vec2i vec0,Vec2i vec1,TGAImage &image,TGAColor color,int y_buffer[] ){
    if(vec0.x>vec1.x)std::swap(vec0,vec1);
    float derror=std::abs((float)(vec1.y-vec0.y)/(vec1.x-vec0.x));
    float error=0;
    int y=vec0.y;
    for(int x=vec0.x;x<vec1.x;x++){
        error+=derror;
        if(error>0.5){
            y+=(vec1.y-vec0.y)>0?1:-1;
            error-=1;
        }
        if(y>y_buffer[x]){
            y_buffer[x]=y;
            image.set(x,0,color);
        }
    }
}
/*
TGAColor getColor_simple(Vec3f* triangle,Vec3f light_dir,TGAColor color){
    Vec3f normal=cross((triangle[2]-triangle[0]),(triangle[1]-triangle[0]));
    normal.normalize();
    float times=normal*light_dir;
    if(times>0)
    return TGAColor(color[0]*times,color[1]*times,color[2]*times,255);
    return TGAColor(0,0,0,255);
}
*/

int main(int argc, char** argv) {
    TGAImage image(width, height, TGAImage::RGB);
    TGAImage texture;
    texture.read_tga_file("obj/african_head/african_head_diffuse.tga");
    texture.flip_vertically();
    float *zbuffer=new float[height*width];
    for(int i=0;i<height*width;i++){
        zbuffer[i]=-std::numeric_limits<int>::max();
    }
    model=new Model("obj/african_head/african_head.obj");
    model=new Model("obj/african_head/african_head_eye_outer.obj");

    for(int i=0;i<model->nfaces();i++){
        std::vector<int> face=model->face(i);
        std::vector<int> texture_count=model->texture(i);
        std::vector<int> norm_count=model->norm(i);
        Vec3f screen_cord[3];
        Vec3f world_cord[3];
        Vec2f texture_cord[3];
        Vec3f norm_cord[3];
        for(int j=0;j<3;j++){
            world_cord[j]=model->vert(face[j]);
            //screen_cord[j]=world2screen(world_cord[j]);
            screen_cord[j]=world_cord[j]*(i==0?width:height);
            texture_cord[j]=model->text(texture_count[j]);
            norm_cord[j]=model->normal(norm_count[j]);
        }
       // triangle(screen_cord,zbuffer,image,getColor_simple(world_cord,Vec3f(0,0,-1),white));
      triangle_texture(screen_cord,texture_cord,norm_cord,zbuffer,image,texture,Vec3f(0,0,-1));
    }
    
    image.flip_vertically();// i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}



