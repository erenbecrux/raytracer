#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define _USE_MATH_DEFINES
#define TINYEXR_IMPLEMENTATION
#define TINYEXR_USE_STB_ZLIB 1
#define TINYEXR_USE_MINIZ 0


#include <iostream>
#include "parser.h"
#include "ppm.h"
#include "stb_image_write.h"
#include <cmath>
#include <limits>
#include <ctime>
#include <thread>
#include <algorithm>
#include "bvh.h"
#include <random>
#include "stb_image.h"
#include <math.h>

#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "tinyexr.h"


using namespace parser;

class Ray {
    public:
        Vec3f origin{};
        Vec3f direction{};
        int reflectionDepth;
        bool isShadowRay;
        float time;
    
    Ray(Vec3f o, Vec3f d, bool shadow) {
        this->origin = o;
        this->direction = d;
        this->isShadowRay = false;
    }
};

class Hit {
    public:
        bool isHit;
        Vec3f hitPoint{};
        float t;
        Vec3f surfaceNormal{};
        Vec3f color{0.0f,0.0f,0.0f};
        int materialID;
        std::vector<int> textureIDs;
        Vec2f uvCoordinates;
        Vec3f tangentVector;
        Vec3f bitangentVector;
        bool isLightHit = false;
};

float DotProduct(const Vec3f &a, const Vec3f &b) {
    float result = a.x * b.x + a.y * b.y + a.z * b.z;
    return result;
}

Vec3f CrossProduct(const Vec3f &a, const Vec3f &b) {
    float i = a.y * b.z - a.z * b.y;
    float j = a.x * b.z - a.z * b.x;
    float k = a.x * b.y - a.y * b.x;
    Vec3f result {i, -j, k};

    return result;
}

Vec3f NormalizeVector3f(const Vec3f &vector) {
    float distance = sqrtf(powf(vector.x,2) + powf(vector.y,2) + powf(vector.z,2));
    Vec3f result{vector.x / distance, vector.y/distance, vector.z/distance};

    return result;
}

Vec3f SubstractVectors(const Vec3f &a, const Vec3f &b) {
    Vec3f result{};
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;

    return result;
}

Vec3f SumVectors(const Vec3f &a, const Vec3f &b) {
    Vec3f result{};
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;

    return result;
}

Vec3f NegateVector(const Vec3f &vector) {
    Vec3f result{};
    result.x = -vector.x;
    result.y = -vector.y;
    result.z = -vector.z;

    return result;
}

Vec3f MultiplyVectorWithConstant(const Vec3f &vector, float c) {
    Vec3f result{};
    result.x = c * vector.x;
    result.y = c * vector.y;
    result.z = c * vector.z;

    return result;
}

float CalculateDeterminant(const Vec3f &a, const Vec3f &b,const Vec3f &c) {
    return (a.x * b.y * c.z) - (a.x * b.z * c.y) - (a.y * b.x * c.z) + (a.y * b.z * c.x) + (a.z * b.x * c.y) - (a.z * b.y * c.x); 
}

float findMin(float x, float y) {
    if(x <= y) {
        return x;
    }
    else {
        return y;
    }
}

float findMax(float x, float y) {
    if(x >= y) {
        return x;
    }
    else {
        return y;
    }
}

void clampColor(Vec3f &color) {
    if(color.x > 255) {
        color.x = 255;
    }

    if(color.y > 255) {
        color.y = 255;
    }

    if(color.z > 255) {
        color.z = 255;
    }
}

Vec3f FindNormal(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{
    return NormalizeVector3f(CrossProduct(SubstractVectors(b,a), SubstractVectors(c,a)));
}


Ray SendRayToPixel(const Camera &cam, int i, int j, float subX, float subY, std::mt19937& mt, std::uniform_real_distribution<float> dist) {
    
    // u = v x w
    Vec3f u = NormalizeVector3f(CrossProduct(cam.up,NegateVector(cam.gaze)));

    Vec3f m = SumVectors(cam.position, MultiplyVectorWithConstant(cam.gaze,cam.near_distance));
    Vec3f q = SumVectors(SumVectors(m, MultiplyVectorWithConstant(u,cam.near_plane.x)), MultiplyVectorWithConstant(cam.up, cam.near_plane.w));

    // near plane: left right bottom top
    float su = (cam.near_plane.y - cam.near_plane.x) * (i + subX) / cam.image_width;
    float sv = (cam.near_plane.w - cam.near_plane.z) * (j + subY) / cam.image_height;

    Vec3f s = SubstractVectors(SumVectors(q,MultiplyVectorWithConstant(u,su)), MultiplyVectorWithConstant(cam.up,sv));
    Vec3f rayDirection = NormalizeVector3f(SubstractVectors(s,cam.position));

    Ray ray {cam.position,rayDirection,false};

    // dof effect
    if(cam.hasDOF) {
        float randDOFx = dist(mt) - 0.5f;
        float randDOFy = dist(mt) - 0.5f;

        Vec3f lensSample = SumVectors(cam.position,SumVectors(MultiplyVectorWithConstant(u, cam.apertureSize * randDOFx),MultiplyVectorWithConstant(cam.up, cam.apertureSize * randDOFy)));

        Vec3f dir = NormalizeVector3f(SubstractVectors(cam.position,s));
        float tDOF = cam.focusDistance / DotProduct(dir,cam.gaze);
        Vec3f pDOF = SumVectors(cam.position,MultiplyVectorWithConstant(dir,tDOF));
        Vec3f dDOF = NormalizeVector3f(SubstractVectors(pDOF,lensSample));

        Ray dofRay {lensSample,dDOF,false};
        return dofRay;
    }
    
    return ray;
}

Hit SphereLightIntersection(const Ray &ray, const LightSphere &sphere, const Vec3f &center) {

    Hit hit {};
    float t;

    Vec3f o_min_c = SubstractVectors(ray.origin,center);

    float A = DotProduct(ray.direction,ray.direction);
    float B = 2 * DotProduct(ray.direction,o_min_c);
    float C = DotProduct(o_min_c,o_min_c) - powf(sphere.radius,2);

    float delta = powf(B,2) - (4*A*C);

    if(delta < 0) {
        hit.isHit = false;
        return hit;
    }
    else if(delta > 0) {
        // double intersection
        float t1 = (-B + sqrt(delta)) / (2 * A);
        float t2 = (-B - sqrt(delta)) / (2 * A);

        if(t1 < 0 && t2 < 0) {
            hit.isHit = false;
            return hit;
        }
        else if(t1 > 0 && t2 > 0) {
            t = findMin(t1,t2);
        }
        else {
            t = findMax(t1,t2);
        }

        Vec3f intersection = SumVectors(ray.origin,MultiplyVectorWithConstant(ray.direction,t));
        hit.isHit = true;
        hit.t = t;
        hit.hitPoint = intersection;
        hit.materialID = sphere.material_id;
        hit.surfaceNormal = NormalizeVector3f(SubstractVectors(intersection,center));
        hit.isLightHit = true;

        return hit;
    }
    else if(delta == 0) {
        // single intersection
        t = -B / (2 * A);

        Vec3f intersection = SumVectors(ray.origin,MultiplyVectorWithConstant(ray.direction,t));
        hit.isHit = true;
        hit.hitPoint = intersection;
        hit.t = t;
        hit.surfaceNormal = NormalizeVector3f(SubstractVectors(intersection,center));
        hit.materialID = sphere.material_id;
        hit.isLightHit = true;

        return hit;
    } 

    // UNKNOWN CASES
    hit.isHit = false;
    return hit;
}

Hit SphereIntersection(const Ray &ray, const Sphere &sphere, const Vec3f &center) {

    Hit hit {};
    float t;

    Vec3f o_min_c = SubstractVectors(ray.origin,center);

    float A = DotProduct(ray.direction,ray.direction);
    float B = 2 * DotProduct(ray.direction,o_min_c);
    float C = DotProduct(o_min_c,o_min_c) - powf(sphere.radius,2);

    float delta = powf(B,2) - (4*A*C);

    if(delta < 0) {
        hit.isHit = false;
        return hit;
    }
    else if(delta > 0) {
        // double intersection
        float t1 = (-B + sqrt(delta)) / (2 * A);
        float t2 = (-B - sqrt(delta)) / (2 * A);

        if(t1 < 0 && t2 < 0) {
            hit.isHit = false;
            return hit;
        }
        else if(t1 > 0 && t2 > 0) {
            t = findMin(t1,t2);
        }
        else {
            t = findMax(t1,t2);
        }

        Vec3f intersection = SumVectors(ray.origin,MultiplyVectorWithConstant(ray.direction,t));
        hit.isHit = true;
        hit.t = t;
        hit.hitPoint = intersection;
        hit.materialID = sphere.material_id;
        hit.surfaceNormal = NormalizeVector3f(SubstractVectors(intersection,center));

        //texture uv calculations
        Vec3f shiftedIntersection;
        shiftedIntersection.x = intersection.x - center.x;
        shiftedIntersection.y = intersection.y - center.y;
        shiftedIntersection.z = intersection.z - center.z;
        float theta = acos(shiftedIntersection.y / sphere.radius);
        float phi = atan2(shiftedIntersection.z,shiftedIntersection.x);
        float u = (-phi + M_PI) / (2*M_PI);
        float v = theta / M_PI;

        hit.uvCoordinates.u = u;
        hit.uvCoordinates.v = v;

        hit.tangentVector.x = 2*M_PI*shiftedIntersection.z;
        hit.tangentVector.y = 0;
        hit.tangentVector.z = -2*M_PI*shiftedIntersection.x;
        hit.tangentVector = hit.tangentVector;

        hit.bitangentVector.x = M_PI*shiftedIntersection.y*cos(phi);
        hit.bitangentVector.y = -sphere.radius*M_PI*sin(theta);
        hit.bitangentVector.z = M_PI*shiftedIntersection.y*sin(phi);
        hit.bitangentVector = hit.bitangentVector;

        for(auto textureid: sphere.textureIDs) {
            hit.textureIDs.push_back(textureid);
        }
    
        return hit;
    }
    else if(delta == 0) {
        // single intersection
        t = -B / (2 * A);

        Vec3f intersection = SumVectors(ray.origin,MultiplyVectorWithConstant(ray.direction,t));
        hit.isHit = true;
        hit.hitPoint = intersection;
        hit.t = t;
        hit.surfaceNormal = NormalizeVector3f(SubstractVectors(intersection,center));
        hit.materialID = sphere.material_id;

        //texture uv calculations
        Vec3f shiftedIntersection;
        shiftedIntersection.x = intersection.x - center.x;
        shiftedIntersection.y = intersection.y - center.y;
        shiftedIntersection.z = intersection.z - center.z;
        float theta = acos(shiftedIntersection.y / sphere.radius);
        float phi = atan2(shiftedIntersection.z,shiftedIntersection.x);
        float u = (-phi + 3.14f) / (2*3.14f);
        float v = theta / 3.14f;

        hit.uvCoordinates.u = u;
        hit.uvCoordinates.v = v;

        hit.tangentVector.x = 2*M_PI*shiftedIntersection.z;
        hit.tangentVector.y = 0;
        hit.tangentVector.z = -2*M_PI*shiftedIntersection.x;
        hit.tangentVector = hit.tangentVector;

        hit.bitangentVector.x = M_PI*shiftedIntersection.y*cos(phi);
        hit.bitangentVector.y = -sphere.radius*M_PI*sin(theta);
        hit.bitangentVector.z = M_PI*shiftedIntersection.y*sin(phi);
        hit.bitangentVector = hit.bitangentVector;

        for(auto textureid: sphere.textureIDs) {
            hit.textureIDs.push_back(textureid);
        }

        return hit;
    } 

    // UNKNOWN CASES
    hit.isHit = false;
    return hit;
}

Hit TriangleIntersection(const Ray &ray, const Vec3f &a, const Vec3f &b, const Vec3f &c, int aID, int bID, int cID, const Scene& scene) {

    Hit hit{};
    hit.surfaceNormal = FindNormal(a,b,c);

    if(DotProduct(hit.surfaceNormal,ray.direction) > 0 && !ray.isShadowRay) {
        hit.isHit = false;
        return hit;
    }

    Vec3f a_minus_c = SubstractVectors(a,c);
    Vec3f b_minus_c = SubstractVectors(b,c);
    Vec3f o_minus_c = SubstractVectors(ray.origin,c);
    Vec3f minusD = NegateVector(ray.direction);


    float detDelta = CalculateDeterminant(a_minus_c,b_minus_c, minusD);

    if(detDelta == 0.0f) {
        hit.isHit = false;
        return hit;
    }

    float detAlpha = CalculateDeterminant(o_minus_c,b_minus_c,minusD);
    float detBeta = CalculateDeterminant(a_minus_c,o_minus_c,minusD);
    float detT = CalculateDeterminant(a_minus_c,b_minus_c,o_minus_c);

    float alpha = detAlpha / detDelta;
    float beta = detBeta / detDelta;
    float t = detT / detDelta;
    float gamma = 1 - alpha - beta;

    if((0 <= alpha && alpha <= 1) && (0 <= beta && beta <= 1) && (0 <= gamma && gamma <= 1) && t > 0) {
        // then this is a hit!
        Vec3f intersection = SumVectors(ray.origin, MultiplyVectorWithConstant(ray.direction, t));
        hit.isHit = true;
        hit.hitPoint = intersection;
        hit.t = t;

        if(scene.texCoord_data.size() > 0) {
            Vec2f tex0 = scene.texCoord_data[aID - 1];
            Vec2f tex1 = scene.texCoord_data[bID - 1];
            Vec2f tex2 = scene.texCoord_data[cID - 1];

            hit.uvCoordinates.u = tex0.u + beta * (tex1.u - tex0.u) + gamma * (tex2.u - tex0.u);
            hit.uvCoordinates.v = tex0.v + beta * (tex1.v - tex0.v) + gamma * (tex2.v - tex0.v);
        }
        
        return hit;
    }

    // No hit!
    hit.isHit = false;
    return hit;
}

Hit MeshLightInterSection(const Ray &ray, const LightMesh &mesh, const Scene &scene) {
    
    Hit finalHit{};
    finalHit.isHit = false;
    finalHit.t = std::numeric_limits<float>::max();

    for(int i = 0; i < mesh.faces.size(); i ++) {

        Face currentFace = mesh.faces[i];
        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        Hit currentHit = TriangleIntersection(ray,v0,v1,v2,currentFace.v0_id,currentFace.v1_id,currentFace.v2_id,scene);

        if(currentHit.isHit) {
            if(currentHit.t < finalHit.t) {
                if(ray.isShadowRay) {
                    finalHit = currentHit;
                    return finalHit;
                }
                currentHit.materialID = mesh.material_id;

                finalHit = currentHit;
                finalHit.isLightHit = true;
            }
        }
    }

    return finalHit;
}

bool BVHIntersection(const Ray &ray, BVHNode* bvh, Hit &hit, const Scene &scene);
Hit TriangleTangentAndBitangentVectors(Hit& hit, Face& face, const Scene& scene);

Hit FindClosestRayIntersection(const Ray &ray,const Scene &scene, std::vector<BVHNode*> bvh) {

    Hit closestHit{};
    closestHit.isHit = false;
    closestHit.t = std::numeric_limits<float>::max();


    // send rays to the spheres
    for(int i = 0; i < scene.spheres.size(); i++) {
        Sphere currentSphere = scene.spheres[i];
        Matrix4x4 transform = currentSphere.transformationMatrix;
        transform = transform.inverse();

        Ray transformRay = ray;
        transformRay.origin = transform.MultiplicationWithPoint(ray.origin);
        transformRay.direction = transform.MultiplicationWithVector(ray.direction);

        Hit currentHit = SphereIntersection(transformRay,currentSphere,scene.vertex_data[currentSphere.center_vertex_id - 1]);
        
        if(currentHit.isHit && currentHit.t < closestHit.t) {
            currentHit.hitPoint = currentSphere.transformationMatrix.MultiplicationWithPoint(currentHit.hitPoint);
            currentHit.surfaceNormal = NormalizeVector3f(transform.transpose().MultiplicationWithVector(currentHit.surfaceNormal));
            closestHit = currentHit;
        }
    }

    // send rays to the triangles
    for(int i = 0; i < scene.triangles.size(); i++) {
        Face currentTriangle = scene.triangles[i].indices;
        Matrix4x4 transform = currentTriangle.transformationMatrix;
        transform = transform.inverse();

        Ray transformRay = ray;
        transformRay.origin = transform.MultiplicationWithPoint(ray.origin);
        transformRay.direction = transform.MultiplicationWithVector(ray.direction);

        Hit currentHit = TriangleIntersection(transformRay,scene.vertex_data[currentTriangle.v0_id - 1] , scene.vertex_data[currentTriangle.v1_id - 1], scene.vertex_data[currentTriangle.v2_id - 1],currentTriangle.v0_id,currentTriangle.v1_id,currentTriangle.v2_id,scene);
        currentHit.hitPoint = currentTriangle.transformationMatrix.MultiplicationWithPoint(currentHit.hitPoint);
        currentHit.surfaceNormal = NormalizeVector3f(transform.transpose().MultiplicationWithVector(currentHit.surfaceNormal));
        if(currentHit.isHit && currentHit.t < closestHit.t) {
            closestHit = currentHit;
            closestHit.materialID = scene.triangles[i].material_id;
            if(scene.texCoord_data.size() > 0) {
                Hit normalTextureHit = TriangleTangentAndBitangentVectors(closestHit,currentTriangle,scene);
                closestHit = normalTextureHit;
            }
        }
    }

    for(int i = 0; i < bvh.size(); i++) {

        if(bvh[i]) {
            Hit bvhHit;
            bvhHit.isHit = false;
            bvhHit.t = std::numeric_limits<float>::max();
            bool isBvhHit = BVHIntersection(ray,bvh[i],bvhHit,scene);
            if(isBvhHit && bvhHit.isHit && bvhHit.t < closestHit.t) {
                closestHit = bvhHit;
            }
        }
    }

    // send rays to the sphere lights
    for(int i = 0; i < scene.lightSpheres.size(); i++) {
        LightSphere currentSphere = scene.lightSpheres[i];
        Matrix4x4 transform = currentSphere.transformationMatrix;
        transform = transform.inverse();

        Ray transformRay = ray;
        transformRay.origin = transform.MultiplicationWithPoint(ray.origin);
        transformRay.direction = transform.MultiplicationWithVector(ray.direction);

        Hit currentHit = SphereLightIntersection(transformRay,currentSphere,scene.vertex_data[currentSphere.center_vertex_id - 1]);
        
        if(currentHit.isHit && currentHit.t < closestHit.t) {
            currentHit.hitPoint = currentSphere.transformationMatrix.MultiplicationWithPoint(currentHit.hitPoint);
            currentHit.surfaceNormal = NormalizeVector3f(transform.transpose().MultiplicationWithVector(currentHit.surfaceNormal));
            closestHit = currentHit;
            closestHit.color = currentSphere.radiance;
        }
    }

    // send rays to the mesh lights
    for(int i = 0; i < scene.lightMeshes.size(); i++) {
        LightMesh currentMesh = scene.lightMeshes[i];
        Matrix4x4 transform = currentMesh.transformationMatrix;
        transform = transform.inverse();

        Ray transformRay = ray;
        transformRay.origin = transform.MultiplicationWithPoint(ray.origin);
        transformRay.direction = transform.MultiplicationWithVector(ray.direction);

        Hit currentHit = MeshLightInterSection(transformRay,currentMesh,scene);
        
        if(currentHit.isHit && currentHit.t < closestHit.t) {
            currentHit.hitPoint = currentMesh.transformationMatrix.MultiplicationWithPoint(currentHit.hitPoint);
            currentHit.surfaceNormal = NormalizeVector3f(transform.transpose().MultiplicationWithVector(currentHit.surfaceNormal));
            closestHit = currentHit;
            closestHit.color = currentMesh.radiance;
        }
    }
    

    return closestHit;
    
}

bool ShadowTest(const Ray &ray, const Hit &currentHit,const Scene &scene,const PointLight &light, std::vector<BVHNode*> bvh) {
    // returns true if the intersection point is in shadow for current light

    Vec3f epsilonPoint = MultiplyVectorWithConstant(currentHit.surfaceNormal,scene.shadow_ray_epsilon);
    Vec3f shadowRayOrigin = SumVectors(epsilonPoint,currentHit.hitPoint);

    Vec3f shadowRayDirection = NormalizeVector3f(SubstractVectors(light.position,currentHit.hitPoint));
    float distance = sqrtf(powf(light.position.x - currentHit.hitPoint.x,2) + powf(light.position.y - currentHit.hitPoint.y,2) + powf(light.position.z - currentHit.hitPoint.z,2));

    Ray shadowRay{shadowRayOrigin,shadowRayDirection,true};
    shadowRay.time = ray.time;

    Hit shadowHit;
    shadowHit.isHit = false;
    shadowHit.t = std::numeric_limits<float>::max();
    //bool isShadowHit = BVHIntersection(shadowRay,bvh,shadowHit,scene);

    shadowHit = FindClosestRayIntersection(shadowRay,scene,bvh);

    if( shadowHit.isHit && shadowHit.t < distance) {
        //std::cout << shadowHit.t << "*" << distance << "\n";
        return true;
    }
    
    return false;
}

Vec3f calculateTextureColor(Vec2f& uvCoords, Texture& texture, const Scene& scene);
Vec3f PerlinNoiseForPoint(Vec3f& p, int conversion, int noiseScale);

Vec3f DiffuseShading(Hit &hit,const PointLight &light, const Scene &scene) {

    Vec3f incomingRadiance{0,0,0};
    Material hitMaterial = scene.materials[hit.materialID - 1];
    float distance = sqrtf(powf(light.position.x - hit.hitPoint.x,2) + powf(light.position.y - hit.hitPoint.y,2) + powf(light.position.z - hit.hitPoint.z,2));
    float cosTheta = DotProduct(NormalizeVector3f(SubstractVectors(light.position,hit.hitPoint)),hit.surfaceNormal);

    if(cosTheta < 0.0f) {
        return incomingRadiance;
    }

    if(hit.textureIDs.size() > 0) {

        for(int i = 0; i < hit.textureIDs.size(); i++) {
            Texture currentTexture = scene.textures[hit.textureIDs[i] - 1];
            if(currentTexture.decalMode == 1) {
                Vec3f texColor;

                if(currentTexture.type == 0) {
                    Vec2f uvCoords = hit.uvCoordinates;
                    texColor = calculateTextureColor(uvCoords,currentTexture,scene);
                    texColor.x = texColor.x / 255;
                    texColor.y = texColor.y / 255;
                    texColor.z = texColor.z / 255;
                }
                else if(currentTexture.type == 1) {
                    texColor = PerlinNoiseForPoint(hit.hitPoint,currentTexture.noiseConversion,currentTexture.noiseScale);
                }
                

                incomingRadiance.x = (light.intensity.x / powf(distance,2)) * cosTheta * texColor.x;
                incomingRadiance.y = (light.intensity.y / powf(distance,2)) * cosTheta * texColor.y;
                incomingRadiance.z = (light.intensity.z / powf(distance,2)) * cosTheta * texColor.z;
                break;
            }
            else if(currentTexture.decalMode == 3){
                Vec3f texColor;

                if(currentTexture.type == 0) {
                    Vec2f uvCoords = hit.uvCoordinates;
                    texColor = calculateTextureColor(uvCoords,currentTexture,scene);
                    texColor.x = texColor.x / 255;
                    texColor.y = texColor.y / 255;
                    texColor.z = texColor.z / 255;
                }
                else if(currentTexture.type == 1) {
                    texColor = PerlinNoiseForPoint(hit.hitPoint,currentTexture.noiseConversion,currentTexture.noiseScale);
                }
                
                incomingRadiance.x = (light.intensity.x / powf(distance,2)) * cosTheta * (((texColor.x) / 2) + (hitMaterial.diffuse.x / 2));
                incomingRadiance.y = (light.intensity.y / powf(distance,2)) * cosTheta * (((texColor.y) / 2) + (hitMaterial.diffuse.y / 2));
                incomingRadiance.z = (light.intensity.z / powf(distance,2)) * cosTheta * (((texColor.z) / 2) + (hitMaterial.diffuse.z / 2));
                break;
            }
            else {
                incomingRadiance.x = (light.intensity.x / powf(distance,2)) * cosTheta * hitMaterial.diffuse.x;
                incomingRadiance.y = (light.intensity.y / powf(distance,2)) * cosTheta * hitMaterial.diffuse.y;
                incomingRadiance.z = (light.intensity.z / powf(distance,2)) * cosTheta * hitMaterial.diffuse.z;
            }
        }        
    }
    else {
        incomingRadiance.x = (light.intensity.x / powf(distance,2)) * cosTheta * hitMaterial.diffuse.x;
        incomingRadiance.y = (light.intensity.y / powf(distance,2)) * cosTheta * hitMaterial.diffuse.y;
        incomingRadiance.z = (light.intensity.z / powf(distance,2)) * cosTheta * hitMaterial.diffuse.z;
    }

    return incomingRadiance;
}

Vec3f SpecularShading(Ray &ray, Hit &hit, const PointLight &light, const Scene &scene) {
    // blinn-phong model

    Vec3f incomingRadiance{0,0,0};
    Material hitMaterial = scene.materials[hit.materialID - 1];

    Vec3f wIncoming = NormalizeVector3f(SubstractVectors(light.position,hit.hitPoint));
    Vec3f wOutgoing = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));

    float cosTheta = DotProduct(wIncoming,hit.surfaceNormal);

    if(cosTheta < 0.0f) {
        return incomingRadiance;
    }

    Vec3f halfVector{};
    halfVector = NormalizeVector3f(SumVectors(wOutgoing,wIncoming));

    float cosAlpha = DotProduct(halfVector,hit.surfaceNormal);
    if(cosAlpha <= 0) {cosAlpha = 0.0f;}

    float phongAlpha;
    if(hitMaterial.has_phong) {
        phongAlpha = powf(cosAlpha,hitMaterial.phong_exponent);
    }
    else {
        phongAlpha = powf(cosAlpha,1);
    }
    

    float distance = sqrtf(powf(light.position.x - hit.hitPoint.x,2) + powf(light.position.y - hit.hitPoint.y,2) + powf(light.position.z - hit.hitPoint.z,2));


    if(hit.textureIDs.size() > 0) {

        for(int i = 0; i < hit.textureIDs.size(); i++) {
            Texture currentTexture = scene.textures[hit.textureIDs[i] - 1];
            if(currentTexture.decalMode == 2) {
                Vec3f texColor;

                if(currentTexture.type == 0) {
                    Vec2f uvCoords = hit.uvCoordinates;
                    texColor = calculateTextureColor(uvCoords,currentTexture,scene);
                    texColor.x = texColor.x / 255;
                    texColor.y = texColor.y / 255;
                    texColor.z = texColor.z / 255;
                }
                else if(currentTexture.type == 1) {
                    texColor = PerlinNoiseForPoint(hit.hitPoint,currentTexture.noiseConversion,currentTexture.noiseScale);
                }

                incomingRadiance.x = (light.intensity.x / powf(distance,2)) * phongAlpha * texColor.x;
                incomingRadiance.y = (light.intensity.y / powf(distance,2)) * phongAlpha * texColor.y;
                incomingRadiance.z = (light.intensity.z / powf(distance,2)) * phongAlpha * texColor.z;
                break;
            } 
            else {
                incomingRadiance.x = (light.intensity.x / powf(distance,2)) * phongAlpha * hitMaterial.specular.x;
                incomingRadiance.y = (light.intensity.y / powf(distance,2)) * phongAlpha * hitMaterial.specular.y;
                incomingRadiance.z = (light.intensity.z / powf(distance,2)) * phongAlpha * hitMaterial.specular.z;
            }
        }        
    }
    else {
        incomingRadiance.x = (light.intensity.x / powf(distance,2)) * phongAlpha * hitMaterial.specular.x;
        incomingRadiance.y = (light.intensity.y / powf(distance,2)) * phongAlpha * hitMaterial.specular.y;
        incomingRadiance.z = (light.intensity.z / powf(distance,2)) * phongAlpha * hitMaterial.specular.z;
    }

    return incomingRadiance;
}

Vec3f AmbientShading(Hit &hit,const Scene &scene) {

    if(scene.hasAmbientLight) {
        Vec3f incomingRadiance{};
        Material hitMaterial = scene.materials[hit.materialID - 1];

        incomingRadiance.x = scene.ambient_light.x * hitMaterial.ambient.x;
        incomingRadiance.y = scene.ambient_light.y * hitMaterial.ambient.y;
        incomingRadiance.z = scene.ambient_light.z * hitMaterial.ambient.z;

        return incomingRadiance;
    }
    else {
        return Vec3f{0,0,0};
    }

}

Ray MirrorReflection(Ray &ray, Hit &hit, const Scene &scene,std::mt19937& mt, std::uniform_real_distribution<float>& dist) {

    Vec3f epsilonedOrigin = SumVectors(MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon),hit.hitPoint);
    Vec3f incomingDirection = NegateVector(ray.direction);
    float cosTheta = DotProduct(incomingDirection,hit.surfaceNormal);
    Vec3f reflectionDirection = NormalizeVector3f(SubstractVectors(MultiplyVectorWithConstant(hit.surfaceNormal,(2*cosTheta)),incomingDirection));
    
    if(scene.materials[hit.materialID - 1].hasRoughness) {

        // create orthonormal basis
        Vec3f nPrime = hit.surfaceNormal;

        // Select a vector not parallel to the normal
        if (std::abs(hit.surfaceNormal.x) <= std::abs(hit.surfaceNormal.y) && std::abs(hit.surfaceNormal.x) <= std::abs(hit.surfaceNormal.z)) {
            nPrime.x = 1.0f;
        } else if (std::abs(hit.surfaceNormal.y) <= std::abs(hit.surfaceNormal.x) && std::abs(hit.surfaceNormal.y) <= std::abs(hit.surfaceNormal.z)) {
            nPrime.y = 1.0f;
        } else {
            nPrime.z = 1.0f;
        }

        Vec3f u = NormalizeVector3f(CrossProduct(nPrime,hit.surfaceNormal));
        Vec3f v = NormalizeVector3f(CrossProduct(hit.surfaceNormal,u));

        // for random number generation
        // we have dist and mt for [0,1)
        float phiOne = dist(mt) - 0.5f;
        float phiTwo = dist(mt) - 0.5f;

        Vec3f rPrime = SumVectors(reflectionDirection,(MultiplyVectorWithConstant(SumVectors(MultiplyVectorWithConstant(u,phiOne),MultiplyVectorWithConstant(v,phiTwo)),scene.materials[hit.materialID - 1].roughness)));
        reflectionDirection = rPrime;
    }
    
    Ray reflectionRay{epsilonedOrigin,reflectionDirection,false};
    return reflectionRay;
}

Vec3f GetBRDFValue(Ray &ray, Hit &hit, const Scene &scene, Material &mat, BRDF &brdf, Vec3f& wi, std::mt19937& mt, std::uniform_real_distribution<float>& dist) {

    if(brdf.type == 0) {
        // OriginalPhong
        Vec3f wrDirection = NormalizeVector3f(SubstractVectors(MultiplyVectorWithConstant(hit.surfaceNormal,2.0f * DotProduct(wi, hit.surfaceNormal)), wi));
        Vec3f woDirection = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));
        float cosAlpha = DotProduct(wrDirection,woDirection);
        float cosTheta = DotProduct(wi,hit.surfaceNormal);


        if(cosTheta > 0) {

            Vec3f brdfValue;
            brdfValue.x = mat.diffuse.x + (mat.specular.x * (powf(cosAlpha, brdf.exponent) / cosTheta));
            brdfValue.y = mat.diffuse.y + (mat.specular.y * (powf(cosAlpha, brdf.exponent) / cosTheta));
            brdfValue.z = mat.diffuse.z + (mat.specular.z * (powf(cosAlpha, brdf.exponent) / cosTheta));

            return brdfValue;
        }
        else {
            return Vec3f{0,0,0};
        }
    }
    else if(brdf.type == 1) {
        //ModifiedPhong

        Vec3f wrDirection = NormalizeVector3f(SubstractVectors(MultiplyVectorWithConstant(hit.surfaceNormal,2.0f * DotProduct(wi, hit.surfaceNormal)), wi));
        Vec3f woDirection = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));
        float cosAlpha = DotProduct(wrDirection,woDirection);
        float cosTheta = DotProduct(wi,hit.surfaceNormal);


        if(cosTheta > 0) {

            Vec3f brdfValue;

            if(brdf.isNormalized) {
                brdfValue.x = (mat.diffuse.x / M_PI) + (mat.specular.x * ((brdf.exponent + 2) / 2 * M_PI) *(powf(cosAlpha, brdf.exponent)));
                brdfValue.y = (mat.diffuse.y / M_PI) + (mat.specular.y * ((brdf.exponent + 2) / 2 * M_PI) *(powf(cosAlpha, brdf.exponent)));
                brdfValue.z = (mat.diffuse.z / M_PI) + (mat.specular.z * ((brdf.exponent + 2) / 2 * M_PI) *(powf(cosAlpha, brdf.exponent)));
            }
            else {
                brdfValue.x = mat.diffuse.x + (mat.specular.x * (powf(cosAlpha, brdf.exponent)));
                brdfValue.y = mat.diffuse.y + (mat.specular.y * (powf(cosAlpha, brdf.exponent)));
                brdfValue.z = mat.diffuse.z + (mat.specular.z * (powf(cosAlpha, brdf.exponent)));
            }

            return brdfValue;
        }
        else {
            return Vec3f{0,0,0};
        }

    }
    else if(brdf.type == 2) {
        //OriginalBlinnPhong

        Vec3f woDirection = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));
        float cosTheta = DotProduct(wi,hit.surfaceNormal);
        Vec3f halfVector{};
        halfVector = NormalizeVector3f(SumVectors(woDirection,wi));
        float cosAlpha = DotProduct(halfVector,hit.surfaceNormal);

        if(cosTheta > 0) {
            Vec3f brdfValue;

            brdfValue.x = mat.diffuse.x + (mat.specular.x * (powf(cosAlpha, brdf.exponent) / cosTheta));
            brdfValue.y = mat.diffuse.y + (mat.specular.y * (powf(cosAlpha, brdf.exponent) / cosTheta));
            brdfValue.z = mat.diffuse.z + (mat.specular.z * (powf(cosAlpha, brdf.exponent) / cosTheta));

            return brdfValue;
        }
        else {
            return Vec3f{0,0,0};
        }
    }
    else if(brdf.type == 3) {
        //ModifiedBlinnPhong

        Vec3f woDirection = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));
        float cosTheta = DotProduct(wi,hit.surfaceNormal);
        Vec3f halfVector;
        halfVector = NormalizeVector3f(SumVectors(woDirection,wi));
        float cosAlpha = DotProduct(halfVector,hit.surfaceNormal);

        if(cosTheta > 0) {
            Vec3f brdfValue;

            if(brdf.isNormalized) {
                brdfValue.x = (mat.diffuse.x / M_PI) + (mat.specular.x * ((brdf.exponent + 8) / 8 * M_PI) *(powf(cosAlpha, brdf.exponent)));
                brdfValue.y = (mat.diffuse.y / M_PI) + (mat.specular.y * ((brdf.exponent + 8) / 8 * M_PI) *(powf(cosAlpha, brdf.exponent)));
                brdfValue.z = (mat.diffuse.z / M_PI) + (mat.specular.z * ((brdf.exponent + 8) / 8 * M_PI) *(powf(cosAlpha, brdf.exponent)));
            }
            else {
                brdfValue.x = mat.diffuse.x + (mat.specular.x * (powf(cosAlpha, brdf.exponent)));
                brdfValue.y = mat.diffuse.y + (mat.specular.y * (powf(cosAlpha, brdf.exponent)));
                brdfValue.z = mat.diffuse.z + (mat.specular.z * (powf(cosAlpha, brdf.exponent)));
            }
            
            return brdfValue;
        }
        else {
            return Vec3f{0,0,0};
        }
    }
    else if(brdf.type == 4) {
        //TorranceSparrow


        Vec3f woDirection = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));
        float cosTheta = DotProduct(wi,hit.surfaceNormal);
        Vec3f halfVector;
        halfVector = NormalizeVector3f(SumVectors(woDirection,wi));
        float cosAlpha = DotProduct(halfVector,hit.surfaceNormal);
        float cosBeta = DotProduct(woDirection,halfVector);
        float cosPhi = DotProduct(woDirection,hit.surfaceNormal);

        float dTerm = ((brdf.exponent + 2) / (2 * M_PI)) * powf(cosAlpha,brdf.exponent);

        float nDotwh = DotProduct(hit.surfaceNormal,halfVector);
        float nDotwo = DotProduct(hit.surfaceNormal,woDirection);
        float nDotwi = DotProduct(hit.surfaceNormal,wi);
        float woDotwh = DotProduct(woDirection,halfVector);
        float gTerm = std::min(1.0f,std::min((2*nDotwh*nDotwo/woDotwh),(2*nDotwh*nDotwi/woDotwh)));

        //float rZero = ((mat.refractionIndex - 1) * (mat.refractionIndex - 1)) / ((mat.refractionIndex + 1) * (mat.refractionIndex + 1));
        //float fTerm = rZero + ((1 - rZero) * powf((1 - cosBeta),5));

        float n2 = mat.refractionIndex;
        float k2 = mat.absorbtionIndex;
        float Rs = (n2*n2 + k2*k2 - 2*n2*cosTheta + cosTheta*cosTheta) / (n2*n2 + k2*k2 + 2*n2*cosTheta + cosTheta*cosTheta);
        float Rp = ((n2*n2 + k2*k2)*cosTheta*cosTheta - 2*n2*cosTheta + 1) / ((n2*n2 + k2*k2)*cosTheta*cosTheta + 2*n2*cosTheta + 1);
        float fTerm = (Rs + Rp) / 2;

        if(cosTheta > 0) {
            Vec3f brdfValue;


            if(!brdf.isKdFresnel) {
                brdfValue.x = (mat.diffuse.x / M_PI) + (mat.specular.x * ((dTerm*fTerm*gTerm)/(4*cosTheta*cosPhi)));
                brdfValue.y = (mat.diffuse.y / M_PI) + (mat.specular.y * ((dTerm*fTerm*gTerm)/(4*cosTheta*cosPhi)));
                brdfValue.z = (mat.diffuse.z / M_PI) + (mat.specular.z * ((dTerm*fTerm*gTerm)/(4*cosTheta*cosPhi)));
            }
            else {
                brdfValue.x = ((1 - fTerm) * mat.diffuse.x / M_PI) + (mat.specular.x * ((dTerm*fTerm*gTerm)/(4*cosTheta*cosPhi)));
                brdfValue.y = ((1 - fTerm) * mat.diffuse.y / M_PI) + (mat.specular.y * ((dTerm*fTerm*gTerm)/(4*cosTheta*cosPhi)));
                brdfValue.z = ((1 - fTerm) * mat.diffuse.z / M_PI) + (mat.specular.z * ((dTerm*fTerm*gTerm)/(4*cosTheta*cosPhi)));
            }

            return brdfValue;
        }

    }

    return Vec3f{0,0,0};
}

Hit NormalMapping(Hit& hit, Texture& texture, const Scene& scene);
Vec3f BumpMapping(Hit& hit, Texture& texture);

Vec3f ApplyShadings(Ray &ray, Hit &hit, const Scene &scene, std::vector<BVHNode*> bvh, std::mt19937& mt, std::uniform_real_distribution<float>& dist) {

    Vec3f radiance{0.0f,0.0f,0.0f};
    Material currentMaterial = scene.materials[hit.materialID - 1];

    if(ray.reflectionDepth > scene.max_recursion_depth) {
        return radiance;
    }


    if(hit.textureIDs.size() > 0) {

        for(int i = 0; i < hit.textureIDs.size(); i++) {
            Texture currentTexture = scene.textures[hit.textureIDs[i] - 1];
            if(currentTexture.decalMode == 4) {
                //replace_all: disable all shadings and paste texture color

                Vec3f texColor;
                if(currentTexture.type == 0) {
                    Vec2f uvCoords = hit.uvCoordinates;
                    texColor = calculateTextureColor(uvCoords,currentTexture,scene);
                }
                else if(currentTexture.type == 1) {
                    texColor = PerlinNoiseForPoint(hit.hitPoint,currentTexture.noiseConversion,currentTexture.noiseScale);
                }

                radiance.x = texColor.x;
                radiance.y = texColor.y;
                radiance.z = texColor.z;

                return radiance;
            }
            else if(currentTexture.decalMode == 5) {
                Hit normalTextureHit = NormalMapping(hit,currentTexture,scene);
                hit = normalTextureHit;
            }
            else if(currentTexture.decalMode == 6) {
                hit.surfaceNormal = BumpMapping(hit,currentTexture);
            }
        }
    }

    Vec3f ambient = AmbientShading(hit,scene);
    radiance.x += ambient.x;
    radiance.y += ambient.y;
    radiance.z += ambient.z;


    for(int i = 0; i < scene.point_lights.size(); i++) {
        PointLight currentLight = scene.point_lights[i];

        if(ShadowTest(ray,hit,scene,currentLight,bvh) == false) {

            if(currentMaterial.materialBRDFid == -1) {
                Vec3f diffuse = DiffuseShading(hit,currentLight,scene);
                radiance.x += diffuse.x; 
                radiance.y += diffuse.y; 
                radiance.z += diffuse.z; 

                Vec3f specular = SpecularShading(ray,hit,currentLight,scene);
                radiance.x += specular.x;
                radiance.y += specular.y;
                radiance.z += specular.z;
            }
            else{
                BRDF currentBRFD = scene.brdfs[currentMaterial.materialBRDFid - 1];
                Vec3f wi = NormalizeVector3f(SubstractVectors(currentLight.position,hit.hitPoint));

                Vec3f brdfValue = GetBRDFValue(ray,hit,scene,currentMaterial,currentBRFD,wi,mt,dist);
                float cosTheta = DotProduct(wi,hit.surfaceNormal);
                float distanceToLight = sqrtf(powf(currentLight.position.x - hit.hitPoint.x,2) + powf(currentLight.position.y - hit.hitPoint.y,2) + powf(currentLight.position.z - hit.hitPoint.z,2));


                radiance.x += brdfValue.x * cosTheta * currentLight.intensity.x / (powf(distanceToLight,2));
                radiance.y += brdfValue.y * cosTheta * currentLight.intensity.y / (powf(distanceToLight,2));
                radiance.z += brdfValue.z * cosTheta * currentLight.intensity.z / (powf(distanceToLight,2));
            }
            
            
        }
            
    }

    for(int i = 0; i < scene.lightSpheres.size(); i++) {
        LightSphere currentLightSphere = scene.lightSpheres[i];
        Vec3f centerOfSphere = scene.vertex_data[currentLightSphere.center_vertex_id - 1];

        // sampling a random point on the sphere
        float randOne = dist(mt);
        float randTwo = dist(mt);

        float phi = 2.0f * M_PI * randOne;
        float cosTheta = 1.0f - 2.0f * randTwo;
        float sinTheta = sqrtf(std::max(0.0f, 1.0f - cosTheta * cosTheta));

        Vec3f directionOnLocal {sinTheta*cos(phi) , sinTheta*sin(phi),cosTheta};
        Vec3f pointOnLocal = SumVectors(centerOfSphere,MultiplyVectorWithConstant(directionOnLocal, currentLightSphere.radius));
        Vec3f pointOnWorld = currentLightSphere.transformationMatrix.MultiplicationWithPoint(pointOnLocal);
        Vec3f pointOnSphere = pointOnWorld;

        // shadow test
        bool isInShadow = false;
        Vec3f epsilonPoint = MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon);
        Vec3f shadowRayOrigin = SumVectors(epsilonPoint,hit.hitPoint);

        Vec3f shadowRayDirection = NormalizeVector3f(SubstractVectors(pointOnSphere,hit.hitPoint));
        float distance = sqrtf(powf(pointOnSphere.x - hit.hitPoint.x,2) + powf(pointOnSphere.y - hit.hitPoint.y,2) + powf(pointOnSphere.z - hit.hitPoint.z,2));

        Ray shadowRay{shadowRayOrigin,shadowRayDirection,true};
        shadowRay.time = ray.time;

        Hit shadowHit;
        shadowHit.isHit = false;
        shadowHit.t = std::numeric_limits<float>::max();

        shadowHit = FindClosestRayIntersection(shadowRay,scene,bvh);

        if( shadowHit.isHit && shadowHit.t < distance && !shadowHit.isLightHit) {
            isInShadow = true;
        }
        

        if(!isInShadow) {

            BRDF currentBRFD = scene.brdfs[currentMaterial.materialBRDFid - 1];
            Vec3f wi = NormalizeVector3f(SubstractVectors(pointOnSphere,hit.hitPoint));

            Vec3f brdfValue = GetBRDFValue(ray,hit,scene,currentMaterial,currentBRFD,wi,mt,dist);
            float cosTheta = DotProduct(wi,hit.surfaceNormal);
            float distanceToLight = sqrtf(powf(pointOnSphere.x - hit.hitPoint.x,2) + powf(pointOnSphere.y - hit.hitPoint.y,2) + powf(pointOnSphere.z - hit.hitPoint.z,2));

            radiance.x += brdfValue.x * cosTheta * currentLightSphere.radiance.x / (powf(distanceToLight,2));
            radiance.y += brdfValue.y * cosTheta * currentLightSphere.radiance.y / (powf(distanceToLight,2));
            radiance.z += brdfValue.z * cosTheta * currentLightSphere.radiance.z / (powf(distanceToLight,2));
        }
    }

    for(int i = 0; i < scene.lightMeshes.size(); i++) {
        LightMesh currentLightMesh = scene.lightMeshes[i];

        std::vector<float> faceAreas(currentLightMesh.faces.size());
        
        float totalArea = 0.0f;
        for(int i = 0; i < currentLightMesh.faces.size(); i++) {
            Face currentFace = currentLightMesh.faces[i];
            
            Vec3f v0F = scene.vertex_data[currentFace.v0_id - 1];
            Vec3f v1F = scene.vertex_data[currentFace.v1_id - 1];
            Vec3f v2F = scene.vertex_data[currentFace.v2_id - 1];

            Vec3f v0World = currentLightMesh.transformationMatrix.MultiplicationWithPoint(v0F);
            Vec3f v1World = currentLightMesh.transformationMatrix.MultiplicationWithPoint(v1F);
            Vec3f v2World = currentLightMesh.transformationMatrix.MultiplicationWithPoint(v2F);

            Vec3f e1 = SubstractVectors(v1World,v2World);
            Vec3f e2 = SubstractVectors(v2World,v0World);
            Vec3f crossEdge = CrossProduct(e1,e2);
            float crossDist = sqrtf(crossEdge.x*crossEdge.x + crossEdge.y*crossEdge.y + crossEdge.z*crossEdge.z);
            float area = 0.5f * crossDist;

            faceAreas[i] = area;
            totalArea += area;
        }

        // compute CDF
        std::vector<float> cdfVector(faceAreas.size());
        float total = 0.0f;
        for(int i = 0; i < faceAreas.size(); i++) {
            total += faceAreas[i];
            cdfVector[i] = total;
        }

        float randOne = dist(mt);
        float r = randOne * totalArea;
        int triangleIndex = 0;
        for(int i = 0; i < cdfVector.size(); i++) {
            if (r <= cdfVector[i]) {
                triangleIndex = i;
                break;
            }
        }

        Face randomFace = currentLightMesh.faces[triangleIndex];
        Vec3f randV0 = scene.vertex_data[randomFace.v0_id - 1];
        Vec3f randV1 = scene.vertex_data[randomFace.v1_id - 1];
        Vec3f randV2 = scene.vertex_data[randomFace.v2_id - 1];

        // random point on triangle
        float phiOne = dist(mt);
        float phiTwo = dist(mt);
        if (phiOne + phiTwo > 1.0f) {
            phiOne = 1.0f - phiOne;
            phiTwo = 1.0f - phiTwo;
        }

        Vec3f pointOnTriangle;
        pointOnTriangle.x = randV0.x + (randV1.x - randV0.x)*phiOne + (randV2.x - randV0.x)*phiTwo;
        pointOnTriangle.y = randV0.y + (randV1.y - randV0.y)*phiOne + (randV2.y - randV0.y)*phiTwo;
        pointOnTriangle.z = randV0.z + (randV1.z - randV0.z)*phiOne + (randV2.z - randV0.z)*phiTwo;

        Vec3f lightToIntersection = SubstractVectors(pointOnTriangle, hit.hitPoint);
        float lightDistance = sqrtf(lightToIntersection.x*lightToIntersection.x + lightToIntersection.y*lightToIntersection.y + lightToIntersection.z*lightToIntersection.z);
        Vec3f wi = NormalizeVector3f(lightToIntersection);

        // shadow test
        bool isInShadow = false;
        Vec3f epsilonPoint = MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon);
        Vec3f shadowRayOrigin = SumVectors(epsilonPoint,hit.hitPoint);

        Vec3f shadowRayDirection = NormalizeVector3f(SubstractVectors(pointOnTriangle,hit.hitPoint));
        float distance = sqrtf(powf(pointOnTriangle.x - hit.hitPoint.x,2) + powf(pointOnTriangle.y - hit.hitPoint.y,2) + powf(pointOnTriangle.z - hit.hitPoint.z,2));

        Ray shadowRay{shadowRayOrigin,shadowRayDirection,true};
        shadowRay.time = ray.time;

        Hit shadowHit;
        shadowHit.isHit = false;
        shadowHit.t = std::numeric_limits<float>::max();

        shadowHit = FindClosestRayIntersection(shadowRay,scene,bvh);

        if( shadowHit.isHit && shadowHit.t < distance && !shadowHit.isLightHit) {
            isInShadow = true;
        }
        

        if(!isInShadow) {
            BRDF currentBRFD = scene.brdfs[currentMaterial.materialBRDFid - 1];

            Vec3f brdfValue = GetBRDFValue(ray,hit,scene,currentMaterial,currentBRFD,wi,mt,dist);
            float cosTheta = DotProduct(wi,hit.surfaceNormal);
            float distanceToLight = sqrtf(powf(wi.x - hit.hitPoint.x,2) + powf(wi.y - hit.hitPoint.y,2) + powf(wi.z - hit.hitPoint.z,2));

            radiance.x += brdfValue.x * cosTheta * currentLightMesh.radiance.x / (powf(distanceToLight,2));
            radiance.y += brdfValue.y * cosTheta * currentLightMesh.radiance.y / (powf(distanceToLight,2));
            radiance.z += brdfValue.z * cosTheta * currentLightMesh.radiance.z / (powf(distanceToLight,2));
        }
    }

    for(int i = 0; i < scene.area_lights.size(); i++) {
        AreaLight currentAreaLight = scene.area_lights[i];

        // set min component to 1
        Vec3f nPrime = currentAreaLight.normal;

        // Select a vector not parallel to the normal
        if (std::abs(currentAreaLight.normal.x) <= std::abs(currentAreaLight.normal.y) && std::abs(currentAreaLight.normal.x) <= std::abs(currentAreaLight.normal.z)) {
            nPrime.x = 1.0f;
        } else if (std::abs(currentAreaLight.normal.y) <= std::abs(currentAreaLight.normal.x) && std::abs(currentAreaLight.normal.y) <= std::abs(currentAreaLight.normal.z)) {
            nPrime.y = 1.0f;
        } else {
            nPrime.z = 1.0f;
        }

        Vec3f u = NormalizeVector3f(CrossProduct(nPrime,currentAreaLight.normal));
        Vec3f v = NormalizeVector3f(CrossProduct(currentAreaLight.normal,u));

        // for random number generation
        // we have dist and mt for [0,1)
        float phiOne = dist(mt) - 0.5f;
        float phiTwo = dist(mt) - 0.5f;

        // random point
        Vec3f pointOnArea = currentAreaLight.position;
        pointOnArea = SumVectors(pointOnArea,SumVectors(MultiplyVectorWithConstant(u, currentAreaLight.extent * phiOne),MultiplyVectorWithConstant(v, currentAreaLight.extent * phiTwo)));

        // shadow test
        // true if the intersection point is in shadow for current light
        bool isShadow = false;
        Vec3f epsilonPoint = MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon);
        Vec3f shadowRayOrigin = SumVectors(epsilonPoint,hit.hitPoint);

        Vec3f shadowRayDirection = NormalizeVector3f(SubstractVectors(pointOnArea,hit.hitPoint));
        float distance = sqrtf(powf(pointOnArea.x - hit.hitPoint.x,2) + powf(pointOnArea.y - hit.hitPoint.y,2) + powf(pointOnArea.z - hit.hitPoint.z,2));

        Ray shadowRay{shadowRayOrigin,shadowRayDirection,true};
        shadowRay.time = ray.time;

        Hit shadowHit;
        shadowHit.isHit = false;
        shadowHit.t = std::numeric_limits<float>::max();
        shadowHit = FindClosestRayIntersection(shadowRay,scene,bvh);

        if( shadowHit.isHit && shadowHit.t < distance) {
            isShadow = true;
        }
        
        // compute shading
        if(!isShadow) {

            float cosThetaArea = abs(DotProduct(NormalizeVector3f(SubstractVectors(hit.hitPoint,pointOnArea)),currentAreaLight.normal));
            float dwi = cosThetaArea * currentAreaLight.extent * currentAreaLight.extent / (distance*distance);

            // diffuse radiance
            Vec3f wi = NormalizeVector3f(SubstractVectors(pointOnArea,hit.hitPoint));

            float cosThetaForDiffuse = DotProduct(NormalizeVector3f(SubstractVectors(pointOnArea,hit.hitPoint)),hit.surfaceNormal);
            float diffuseDWI = dwi;
            if (cosThetaForDiffuse < 0.0f)
            {
                diffuseDWI = 0.0f;  // Avoid negative or invalid values
            }

		    
            radiance.x += currentAreaLight.radiance.x * diffuseDWI * cosThetaForDiffuse * currentMaterial.diffuse.x;
            radiance.y += currentAreaLight.radiance.y * diffuseDWI * cosThetaForDiffuse * currentMaterial.diffuse.y;
            radiance.z += currentAreaLight.radiance.z * diffuseDWI * cosThetaForDiffuse * currentMaterial.diffuse.z;

            
            // specular radiance
            
            Vec3f wIncoming = NormalizeVector3f(SubstractVectors(pointOnArea,hit.hitPoint));
            Vec3f wOutgoing = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));
            float cosThetaForSpec = DotProduct(wIncoming,hit.surfaceNormal);
            float specularDWI = dwi;
            if(cosThetaForSpec < 0.0f) {
                specularDWI = 0.0f;
            }

            Vec3f halfVector{};
            halfVector = NormalizeVector3f(SumVectors(wOutgoing,wIncoming));

            float cosAlpha = DotProduct(halfVector,hit.surfaceNormal);
            if(cosAlpha <= 0) {cosAlpha = 0.0f;}

            float phongAlpha;
            if(currentMaterial.has_phong) {
                phongAlpha = powf(cosAlpha,currentMaterial.phong_exponent);
            }
            else {
                phongAlpha = powf(cosAlpha,1);
            }

            radiance.x += currentAreaLight.radiance.x * specularDWI * phongAlpha * currentMaterial.specular.x;
            radiance.y += currentAreaLight.radiance.y * specularDWI * phongAlpha * currentMaterial.specular.y;
            radiance.z += currentAreaLight.radiance.z * specularDWI * phongAlpha * currentMaterial.specular.z;
            
        }

    }

    for(int i = 0; i < scene.directional_lights.size(); i++) {
        DirectionalLight currentDirectionalLight = scene.directional_lights[i];

        Vec3f wIncoming = NormalizeVector3f(NegateVector(currentDirectionalLight.direction));

        // shadow test
        Vec3f epsilonPoint = MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon);
        Vec3f shadowRayOrigin = SumVectors(epsilonPoint,hit.hitPoint);

        Vec3f shadowRayDirection = wIncoming;
        Ray shadowRay{shadowRayOrigin,shadowRayDirection,true};
        shadowRay.time = ray.time;

        Hit shadowHit;
        shadowHit.isHit = false;
        shadowHit.t = std::numeric_limits<float>::max();
        shadowHit = FindClosestRayIntersection(shadowRay,scene,bvh);

        bool isInShadow = false;
        if( shadowHit.isHit && shadowHit.t < std::numeric_limits<float>::max()) {
            isInShadow = true;
        }
        
        if(!isInShadow) {
            // diffuse
            Vec3f incomingRadiance{0,0,0};
            Material hitMaterial = scene.materials[hit.materialID - 1];

            float distance = std::numeric_limits<float>::max(); // infinity
            float cosTheta = DotProduct(wIncoming,hit.surfaceNormal);

            Vec3f diffuseRadiance; // = currentDirectionalLight.radiance * kd * cosTheta
            diffuseRadiance.x = currentDirectionalLight.radiance.x * hitMaterial.diffuse.x * cosTheta;
            diffuseRadiance.y = currentDirectionalLight.radiance.y * hitMaterial.diffuse.y * cosTheta;
            diffuseRadiance.z = currentDirectionalLight.radiance.z * hitMaterial.diffuse.z * cosTheta;

            radiance.x += diffuseRadiance.x;
            radiance.y += diffuseRadiance.y;
            radiance.z += diffuseRadiance.z;


            // specular
            Vec3f halfVector{};
            Vec3f wOutgoing = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));
            halfVector = NormalizeVector3f(SumVectors(wOutgoing,wIncoming));

            float cosAlpha = DotProduct(halfVector,hit.surfaceNormal);
            if(cosAlpha <= 0) {cosAlpha = 0.0f;}

            float phongAlpha;
            if(hitMaterial.has_phong) {
                phongAlpha = powf(cosAlpha,hitMaterial.phong_exponent);
            }
            else {
                phongAlpha = powf(cosAlpha,1);
            }

            Vec3f specularRadiance; // = currentDirectionalLight.radiance * ks * phongAlpha
            specularRadiance.x = currentDirectionalLight.radiance.x * hitMaterial.specular.x * phongAlpha;
            specularRadiance.y = currentDirectionalLight.radiance.y * hitMaterial.specular.y * phongAlpha;
            specularRadiance.z = currentDirectionalLight.radiance.z * hitMaterial.specular.z * phongAlpha;

            radiance.x += specularRadiance.x;
            radiance.y += specularRadiance.y;
            radiance.z += specularRadiance.z;
        }
    }

    for(int i = 0; i < scene.spot_lights.size(); i++) {
        SpotLight currentSpotLight = scene.spot_lights[i];
        float falloffAngleRad = (currentSpotLight.falloffAngle * M_PI) / 180.0f;
        float coverageAngleRad = (currentSpotLight.coverageAngle * M_PI) / 180.0f;

        Vec3f intersectionToLight = NormalizeVector3f(SubstractVectors(currentSpotLight.position,hit.hitPoint));
        float spotCosAlpha = DotProduct(NegateVector(intersectionToLight),NormalizeVector3f(currentSpotLight.direction));
        float spotAlpha = acos(spotCosAlpha);

        float insideOfS = (spotCosAlpha - cos(coverageAngleRad / 2)) / (cos(falloffAngleRad / 2) - cos(coverageAngleRad / 2));
        float spotAttenuationS = pow(insideOfS,4);

        // calculate net irradiance
        Vec3f netRadianceSpotLight;
        float distance = sqrtf(powf(currentSpotLight.position.x - hit.hitPoint.x,2) + powf(currentSpotLight.position.y - hit.hitPoint.y,2) + powf(currentSpotLight.position.z - hit.hitPoint.z,2));
        if(spotAlpha < falloffAngleRad / 2) {
            netRadianceSpotLight.x = currentSpotLight.intensity.x / pow(distance,2);
            netRadianceSpotLight.y = currentSpotLight.intensity.y / pow(distance,2);
            netRadianceSpotLight.z = currentSpotLight.intensity.z / pow(distance,2);
        }
        else if (spotAlpha < coverageAngleRad / 2) {
            netRadianceSpotLight.x = spotAttenuationS * currentSpotLight.intensity.x / pow(distance,2);
            netRadianceSpotLight.y = spotAttenuationS * currentSpotLight.intensity.y / pow(distance,2);
            netRadianceSpotLight.z = spotAttenuationS * currentSpotLight.intensity.z / pow(distance,2);
        }
        else {
            netRadianceSpotLight.x = 0;
            netRadianceSpotLight.y = 0;
            netRadianceSpotLight.z = 0;
        }

        // shadow test
        Vec3f wIncoming = intersectionToLight;
        Vec3f epsilonPoint = MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon);
        Vec3f shadowRayOrigin = SumVectors(epsilonPoint,hit.hitPoint);

        Vec3f shadowRayDirection = wIncoming;
        Ray shadowRay{shadowRayOrigin,shadowRayDirection,true};
        shadowRay.time = ray.time;

        Hit shadowHit;
        shadowHit.isHit = false;
        shadowHit.t = std::numeric_limits<float>::max();
        shadowHit = FindClosestRayIntersection(shadowRay,scene,bvh);

        bool isInShadow = false;
        if( shadowHit.isHit && shadowHit.t < distance) {
            isInShadow = true;
        }
        
        if(!isInShadow) {
            // diffuse
            Vec3f incomingRadiance{0,0,0};
            Material hitMaterial = scene.materials[hit.materialID - 1];

            float cosTheta = DotProduct(wIncoming,hit.surfaceNormal);
            

            Vec3f diffuseRadiance; // = currentDirectionalLight.radiance * kd * cosTheta
            diffuseRadiance.x = netRadianceSpotLight.x * hitMaterial.diffuse.x * cosTheta;
            diffuseRadiance.y = netRadianceSpotLight.y * hitMaterial.diffuse.y * cosTheta;
            diffuseRadiance.z = netRadianceSpotLight.z * hitMaterial.diffuse.z * cosTheta;

            if(cosTheta < 0.0f) {
                diffuseRadiance.x = 0.0f;
                diffuseRadiance.y = 0.0f;
                diffuseRadiance.z = 0.0f;
            }

            radiance.x += diffuseRadiance.x;
            radiance.y += diffuseRadiance.y;
            radiance.z += diffuseRadiance.z;


            // specular
            Vec3f halfVector{};
            Vec3f wOutgoing = NormalizeVector3f(SubstractVectors(ray.origin,hit.hitPoint));
            halfVector = NormalizeVector3f(SumVectors(wOutgoing,wIncoming));

            float cosAlpha = DotProduct(halfVector,hit.surfaceNormal);
            if(cosAlpha <= 0) {cosAlpha = 0.0f;}

            float phongAlpha;
            if(hitMaterial.has_phong) {
                phongAlpha = powf(cosAlpha,hitMaterial.phong_exponent);
            }
            else {
                phongAlpha = powf(cosAlpha,1);
            }

            Vec3f specularRadiance; // = currentDirectionalLight.radiance * ks * phongAlpha
            specularRadiance.x = netRadianceSpotLight.x * hitMaterial.specular.x * phongAlpha;
            specularRadiance.y = netRadianceSpotLight.y * hitMaterial.specular.y * phongAlpha;
            specularRadiance.z = netRadianceSpotLight.z * hitMaterial.specular.z * phongAlpha;

            if(cosTheta < 0.0f) {
                specularRadiance.x = 0.0f;
                specularRadiance.y = 0.0f;
                specularRadiance.z = 0.0f;
            }

            radiance.x += specularRadiance.x;
            radiance.y += specularRadiance.y;
            radiance.z += specularRadiance.z;
        }

    }

    for(int i = 0; i < scene.sphericalDirectional_lights.size(); i++) {
        SphericalDirectionalLight currentSphericalDirectionalLight = scene.sphericalDirectional_lights[i];
        Vec3f vectorForCalculation;
        bool foundAVector = false;

        if(currentMaterial.is_mirror) {
            foundAVector = true;
            vectorForCalculation = MirrorReflection(ray,hit,scene,mt,dist).direction;
        }

        if(currentMaterial.is_dielectric) {
            foundAVector = true;
            float n1 = 1.0f; // vacuum
            float n2 = currentMaterial.refractionIndex;

            Vec3f incomingDirection = NegateVector(ray.direction);
            Vec3f normalDirection = hit.surfaceNormal; // pointing outside
            float cosTheta = DotProduct(incomingDirection,normalDirection);
            Vec3f refractedDirection;


            bool rayIsEntering = cosTheta > 0.0f;
            float insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));
            float cosPhi;

            if(rayIsEntering) {
                cosPhi = sqrtf(insideTerm);
            
                Vec3f dNcosTheta = SumVectors(MultiplyVectorWithConstant(normalDirection,cosTheta),ray.direction);
                refractedDirection.x = dNcosTheta.x * (n1/n2) - normalDirection.x * cosPhi;
                refractedDirection.y = dNcosTheta.y * (n1/n2) - normalDirection.y * cosPhi;
                refractedDirection.z = dNcosTheta.z * (n1/n2) - normalDirection.z * cosPhi;

            }
            else {
                // swap n
                n1 = currentMaterial.refractionIndex;
                n2 = 1.0f; // vacuum
                
                insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));

                if(insideTerm < 0) {
                    n1 = 1.0f;
                    n2 = currentMaterial.refractionIndex;
                    insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));
                }

                cosPhi = sqrtf(insideTerm);

                normalDirection = NegateVector(normalDirection); // pointing inside
                cosTheta = -cosTheta;
                rayIsEntering = !rayIsEntering;

                Vec3f dNcosTheta = SumVectors(MultiplyVectorWithConstant(normalDirection,cosTheta),ray.direction);
                refractedDirection.x = dNcosTheta.x * (n1/n2) - normalDirection.x * cosPhi;
                refractedDirection.y = dNcosTheta.y * (n1/n2) - normalDirection.y * cosPhi;
                refractedDirection.z = dNcosTheta.z * (n1/n2) - normalDirection.z * cosPhi;

                normalDirection = NegateVector(normalDirection); // pointing outside
            }


            vectorForCalculation = refractedDirection;
        }


        while(!foundAVector) {
            float phiOne = dist(mt);
            float phiTwo = dist(mt);
            float phiThree = dist(mt);
            phiOne = 2*phiOne - 1;
            phiTwo = 2*phiTwo - 1;
            phiThree = 2*phiThree - 1;

            Vec3f candidateVector;
            candidateVector.x = phiOne;
            candidateVector.y = phiTwo;
            candidateVector.z = phiThree;

            float lenghtOfCandidateVector = sqrtf(pow(phiOne,2)+pow(phiTwo,2)+pow(phiThree,2));
            if ((lenghtOfCandidateVector <= 1) && (DotProduct(hit.surfaceNormal,candidateVector) > 0)) {
                vectorForCalculation = NormalizeVector3f(candidateVector);
                foundAVector = true;
            }
        }

        float u,v;

        if(currentSphericalDirectionalLight.type == 0) {
            // latlong
            u = ((1 + ( ( atan2(vectorForCalculation.x,-vectorForCalculation.z) / M_PI ) ) ) / 2);
            v = acos(vectorForCalculation.y) / M_PI;
        }
        else if(currentSphericalDirectionalLight.type == 1) {
            // probe
            float r = (1/M_PI) * ( acos(-vectorForCalculation.z) / sqrtf(pow(vectorForCalculation.x,2) + pow(vectorForCalculation.y,2)));
            u = (r*vectorForCalculation.x + 1) / 2;
            v = (-r*vectorForCalculation.y + 1) / 2;
        }


        ImageWithData currentImage = scene.imageDataVector[currentSphericalDirectionalLight.imageID - 1];
        int pixelI = round(u * currentImage.width);
        int pixelJ = round(v * currentImage.height);

        pixelI = pixelI % currentImage.width;
        pixelJ = pixelJ % currentImage.height;
        Vec3f fetchingColor{0,0,0};
        int pixel_index = (pixelJ * currentImage.width + pixelI) * 4;

        fetchingColor.x = currentImage.float_data[pixel_index];
        fetchingColor.y = currentImage.float_data[++pixel_index];
        fetchingColor.z = currentImage.float_data[++pixel_index];

        fetchingColor.x = 2 * M_PI * fetchingColor.x;
        fetchingColor.y = 2 * M_PI * fetchingColor.y;
        fetchingColor.z = 2 * M_PI * fetchingColor.z;

        radiance.x += fetchingColor.x;
        radiance.y += fetchingColor.y;
        radiance.z += fetchingColor.z;
        
    }

    if(currentMaterial.is_mirror && ray.reflectionDepth <= scene.max_recursion_depth) {

        Ray reflectionRay = MirrorReflection(ray,hit,scene,mt,dist);
        Hit reflectionHit = FindClosestRayIntersection(reflectionRay,scene,bvh);
        ray.reflectionDepth += 1;
        reflectionRay.time = ray.time;
        reflectionRay.reflectionDepth = ray.reflectionDepth;
        
        if(reflectionHit.isHit) {

            Vec3f reflectionRadiance{0.0f,0.0f,0.0f};
            reflectionRadiance = ApplyShadings(reflectionRay,reflectionHit,scene,bvh,mt,dist);

            radiance.x += reflectionRadiance.x * currentMaterial.mirror.x;
            radiance.y += reflectionRadiance.y * currentMaterial.mirror.y;
            radiance.z += reflectionRadiance.z * currentMaterial.mirror.z;
        }
    }
    

    if(currentMaterial.is_conductor && ray.reflectionDepth <= scene.max_recursion_depth) {
        float n2 = currentMaterial.refractionIndex;
        float k2 = currentMaterial.absorbtionIndex;

        float cosTheta = -DotProduct(ray.direction,hit.surfaceNormal);
        Vec3f wr = NormalizeVector3f(SubstractVectors(MultiplyVectorWithConstant(hit.surfaceNormal,2*cosTheta),NegateVector(ray.direction)));

        if(currentMaterial.hasRoughness) {
            // create orthonormal basis
            Vec3f nPrime = hit.surfaceNormal;

            // Select a vector not parallel to the normal
            if (std::abs(hit.surfaceNormal.x) <= std::abs(hit.surfaceNormal.y) && std::abs(hit.surfaceNormal.x) <= std::abs(hit.surfaceNormal.z)) {
                nPrime.x = 1.0f;
            } else if (std::abs(hit.surfaceNormal.y) <= std::abs(hit.surfaceNormal.x) && std::abs(hit.surfaceNormal.y) <= std::abs(hit.surfaceNormal.z)) {
                nPrime.y = 1.0f;
            } else {
                nPrime.z = 1.0f;
            }

            Vec3f u = NormalizeVector3f(CrossProduct(nPrime,hit.surfaceNormal));
            Vec3f v = NormalizeVector3f(CrossProduct(hit.surfaceNormal,u));

            // for random number generation
            // we have dist and mt for [0,1)
            float phiOne = dist(mt) - 0.5f;
            float phiTwo = dist(mt) - 0.5f;

            Vec3f rPrime = SumVectors(wr,(MultiplyVectorWithConstant(SumVectors(MultiplyVectorWithConstant(u,phiOne),MultiplyVectorWithConstant(v,phiTwo)),currentMaterial.roughness)));
            wr = rPrime;
        }

        float Rs = (n2*n2 + k2*k2 - 2*n2*cosTheta + cosTheta*cosTheta) / (n2*n2 + k2*k2 + 2*n2*cosTheta + cosTheta*cosTheta);
        float Rp = ((n2*n2 + k2*k2)*cosTheta*cosTheta - 2*n2*cosTheta + 1) / ((n2*n2 + k2*k2)*cosTheta*cosTheta + 2*n2*cosTheta + 1);

        float Fr = (Rs + Rp) / 2;

        Ray reflectionRay{SumVectors(MultiplyVectorWithConstant(hit.surfaceNormal,scene.shadow_ray_epsilon),hit.hitPoint),wr,false};
        Hit reflectionHit = FindClosestRayIntersection(reflectionRay,scene,bvh);
        ray.reflectionDepth += 1;
        reflectionRay.time = ray.time;
        reflectionRay.reflectionDepth = ray.reflectionDepth;
        
        if(reflectionHit.isHit) {
            Vec3f reflectionRadiance = ApplyShadings(reflectionRay,reflectionHit,scene,bvh,mt,dist);

            radiance.x += reflectionRadiance.x * currentMaterial.mirror.x * Fr;
            radiance.y += reflectionRadiance.y * currentMaterial.mirror.y * Fr;
            radiance.z += reflectionRadiance.z * currentMaterial.mirror.z * Fr;
        }   
    }

    if(currentMaterial.is_dielectric && ray.reflectionDepth <= scene.max_recursion_depth) {
        float n1 = 1.0f; // vacuum
        float n2 = currentMaterial.refractionIndex;

        Vec3f incomingDirection = NegateVector(ray.direction);
        Vec3f normalDirection = hit.surfaceNormal; // pointing outside
        float cosTheta = DotProduct(incomingDirection,normalDirection);
        Vec3f reflectedDirection = NormalizeVector3f(SubstractVectors(MultiplyVectorWithConstant(normalDirection,2*cosTheta),incomingDirection));
        Vec3f refractedDirection;


        bool rayIsEntering = cosTheta > 0.0f;
        float insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));
        float cosPhi;

        if(rayIsEntering) {
            cosPhi = sqrtf(insideTerm);
        
            Vec3f dNcosTheta = SumVectors(MultiplyVectorWithConstant(normalDirection,cosTheta),ray.direction);
            refractedDirection.x = dNcosTheta.x * (n1/n2) - normalDirection.x * cosPhi;
            refractedDirection.y = dNcosTheta.y * (n1/n2) - normalDirection.y * cosPhi;
            refractedDirection.z = dNcosTheta.z * (n1/n2) - normalDirection.z * cosPhi;

        }
        else {
            // swap n
            n1 = currentMaterial.refractionIndex;
            n2 = 1.0f; // vacuum
            
            insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));

            if(insideTerm < 0) {
                n1 = 1.0f;
                n2 = currentMaterial.refractionIndex;
                insideTerm = 1 - ((n1/n2) * (n1/n2) * (1 - (cosTheta * cosTheta)));
            }

            cosPhi = sqrtf(insideTerm);

            normalDirection = NegateVector(normalDirection); // pointing inside
            cosTheta = -cosTheta;
            rayIsEntering = !rayIsEntering;

            Vec3f dNcosTheta = SumVectors(MultiplyVectorWithConstant(normalDirection,cosTheta),ray.direction);
            refractedDirection.x = dNcosTheta.x * (n1/n2) - normalDirection.x * cosPhi;
            refractedDirection.y = dNcosTheta.y * (n1/n2) - normalDirection.y * cosPhi;
            refractedDirection.z = dNcosTheta.z * (n1/n2) - normalDirection.z * cosPhi;

            normalDirection = NegateVector(normalDirection); // pointing outside
        }

        if(currentMaterial.hasRoughness) {
            // create orthonormal basis
            Vec3f nPrime = hit.surfaceNormal;

            // Select a vector not parallel to the normal
            if (std::abs(hit.surfaceNormal.x) <= std::abs(hit.surfaceNormal.y) && std::abs(hit.surfaceNormal.x) <= std::abs(hit.surfaceNormal.z)) {
                nPrime.x = 1.0f;
            } else if (std::abs(hit.surfaceNormal.y) <= std::abs(hit.surfaceNormal.x) && std::abs(hit.surfaceNormal.y) <= std::abs(hit.surfaceNormal.z)) {
                nPrime.y = 1.0f;
            } else {
                nPrime.z = 1.0f;
            }

            Vec3f u = NormalizeVector3f(CrossProduct(nPrime,hit.surfaceNormal));
            Vec3f v = NormalizeVector3f(CrossProduct(hit.surfaceNormal,u));

            // for random number generation
            // we have dist and mt for [0,1)
            float phiOne = dist(mt) - 0.5f;
            float phiTwo = dist(mt) - 0.5f;

            Vec3f rPrime = SumVectors(reflectedDirection,(MultiplyVectorWithConstant(SumVectors(MultiplyVectorWithConstant(u,phiOne),MultiplyVectorWithConstant(v,phiTwo)),currentMaterial.roughness)));
            reflectedDirection = rPrime;
        }

        if(currentMaterial.hasRoughness) {
            // create orthonormal basis
            Vec3f nPrime = hit.surfaceNormal;

            // Select a vector not parallel to the normal
            if (std::abs(hit.surfaceNormal.x) <= std::abs(hit.surfaceNormal.y) && std::abs(hit.surfaceNormal.x) <= std::abs(hit.surfaceNormal.z)) {
                nPrime.x = 1.0f;
            } else if (std::abs(hit.surfaceNormal.y) <= std::abs(hit.surfaceNormal.x) && std::abs(hit.surfaceNormal.y) <= std::abs(hit.surfaceNormal.z)) {
                nPrime.y = 1.0f;
            } else {
                nPrime.z = 1.0f;
            }

            Vec3f u = NormalizeVector3f(CrossProduct(nPrime,hit.surfaceNormal));
            Vec3f v = NormalizeVector3f(CrossProduct(hit.surfaceNormal,u));

            // for random number generation
            // we have dist and mt for [0,1)
            float phiOne = dist(mt) - 0.5f;
            float phiTwo = dist(mt) - 0.5f;

            Vec3f rPrime = SumVectors(refractedDirection,(MultiplyVectorWithConstant(SumVectors(MultiplyVectorWithConstant(u,phiOne),MultiplyVectorWithConstant(v,phiTwo)),currentMaterial.roughness)));
            refractedDirection = rPrime;
        }


        float Fr, Ft;
        float rparallel, rperpendicular;

        rparallel = (n2*cosTheta - n1*cosPhi) / (n2*cosTheta + n1*cosPhi);
        rperpendicular = (n1*cosTheta - n2*cosPhi) / (n1*cosTheta + n2*cosPhi);

        Fr = (rparallel * rparallel + rperpendicular * rperpendicular) / 2;
        Ft = 1-Fr;

        if(Fr == 1.0f) {
            // full reflection

            Ray reflectionRay{SumVectors(hit.hitPoint, MultiplyVectorWithConstant(reflectedDirection,scene.shadow_ray_epsilon)),reflectedDirection,false};
            Hit reflectionHit = FindClosestRayIntersection(reflectionRay,scene,bvh);
            ray.reflectionDepth++;
            reflectionRay.time = ray.time;
            reflectionRay.reflectionDepth = ray.reflectionDepth;
            Vec3f reflectionColor{0,0,0};
            if(reflectionHit.isHit) {
                reflectionColor = ApplyShadings(reflectionRay,reflectionHit,scene,bvh,mt,dist);
            }

            radiance.x += (reflectionColor.x * Fr);
            radiance.y += (reflectionColor.y * Fr);
            radiance.z += (reflectionColor.z * Fr);
        }
        else {
            Ray reflectionRay{SumVectors(hit.hitPoint, MultiplyVectorWithConstant(reflectedDirection,scene.shadow_ray_epsilon)),reflectedDirection,false};
            Hit reflectionHit = FindClosestRayIntersection(reflectionRay,scene,bvh);
            ray.reflectionDepth++;
            reflectionRay.time = ray.time;
            reflectionRay.reflectionDepth = ray.reflectionDepth;
            Vec3f reflectionColor{0,0,0};
            if(reflectionHit.isHit) {
                reflectionColor = ApplyShadings(reflectionRay,reflectionHit,scene,bvh,mt,dist);
            } 
            

            Ray refractionRay{SumVectors(hit.hitPoint,MultiplyVectorWithConstant(refractedDirection,scene.shadow_ray_epsilon)),refractedDirection,false};
            Hit refractionHit = FindClosestRayIntersection(refractionRay,scene,bvh);
            ray.reflectionDepth++;
            refractionRay.time = ray.time;
            refractionRay.reflectionDepth = ray.reflectionDepth;
            Vec3f refractionColor{0,0,0};
            if(refractionHit.isHit) {
                refractionColor = ApplyShadings(refractionRay,refractionHit,scene,bvh,mt,dist);
            }  

            float distance = sqrtf(powf(refractionHit.hitPoint.x- hit.hitPoint.x,2) + powf(refractionHit.hitPoint.y- hit.hitPoint.y,2) + powf(refractionHit.hitPoint.z- hit.hitPoint.z,2));

            Vec3f attenuation{0,0,0};

            attenuation.x = refractionColor.x * expf(-currentMaterial.absorbtionCoeff.x * distance);
            attenuation.y = refractionColor.y * expf(-currentMaterial.absorbtionCoeff.y * distance);
            attenuation.z = refractionColor.z * expf(-currentMaterial.absorbtionCoeff.z * distance);

            radiance.x += attenuation.x * Ft  + (reflectionColor.x * Fr);
            radiance.y += attenuation.y * Ft  + (reflectionColor.y * Fr);
            radiance.z += attenuation.z * Ft  + (reflectionColor.z * Fr);
            }

    }

    return radiance;
}

// BVH

BoundingBox TransformBoundingBox(const BoundingBox& box, Matrix4x4& transformationMatrix) {
    // Get the eight corners of the bounding box
    Vec3f corners[8] = {
        {box.min.x, box.min.y, box.min.z},
        {box.max.x, box.min.y, box.min.z},
        {box.min.x, box.max.y, box.min.z},
        {box.min.x, box.min.y, box.max.z},
        {box.max.x, box.max.y, box.min.z},
        {box.max.x, box.min.y, box.max.z},
        {box.min.x, box.max.y, box.max.z},
        {box.max.x, box.max.y, box.max.z}
    };

    // Transform each corner and update the bounding box
    BoundingBox transformedBox;
    transformedBox.min = transformedBox.max = transformationMatrix.MultiplicationWithPoint(corners[0]);

    for (int i = 1; i < 8; ++i) {
        Vec3f transformedCorner = transformationMatrix.MultiplicationWithPoint(corners[i]);

        // Update min and max
        transformedBox.min.x = std::min(transformedBox.min.x, transformedCorner.x);
        transformedBox.min.y = std::min(transformedBox.min.y, transformedCorner.y);
        transformedBox.min.z = std::min(transformedBox.min.z, transformedCorner.z);

        transformedBox.max.x = std::max(transformedBox.max.x, transformedCorner.x);
        transformedBox.max.y = std::max(transformedBox.max.y, transformedCorner.y);
        transformedBox.max.z = std::max(transformedBox.max.z, transformedCorner.z);
    }

    return transformedBox;
}

float RayBoxIntersection(const Ray &ray, const BoundingBox &bb) {

    float t1x = (bb.min.x - ray.origin.x) / ray.direction.x; // min
    float t2x = (bb.max.x - ray.origin.x) / ray.direction.x; // max

    if(t1x > 0 && t2x < t1x) {
        std::swap(t1x,t2x);
    }

    float t1y = (bb.min.y - ray.origin.y) / ray.direction.y; // min
    float t2y = (bb.max.y - ray.origin.y) / ray.direction.y; // max

    if(t1y > 0 && t2y < t1y) {
        std::swap(t1y,t2y);
    }

    float t1z = (bb.min.z - ray.origin.z) / ray.direction.z; // min
    float t2z = (bb.max.z - ray.origin.z) / ray.direction.z; // max

    if(t1z > 0 && t2z < t1z) {
        std::swap(t1z,t2z);
    }


    float maxt1 = std::max(t1x,t1y);
    maxt1 = std::max(maxt1,t1z);

    float mint2 = std::min(t2x,t2y);
    mint2 = std::min(mint2,t2z);

    // Check if the ray origin is inside the box
    bool originInside = 
        ray.origin.x >= bb.min.x && ray.origin.x <= bb.max.x &&
        ray.origin.y >= bb.min.y && ray.origin.y <= bb.max.y &&
        ray.origin.z >= bb.min.z && ray.origin.z <= bb.max.z;

    // If origin is inside, use only the smallest positive t value of exits (mint2)
    if (originInside && mint2 > 0) {
        return mint2;
    }
    
    // Standard outside-ray-box intersection
    if (maxt1 > mint2 || mint2 < 0) {
        return std::numeric_limits<float>::max(); // No valid intersection
    }

    return maxt1;

}

bool BVHIntersection(const Ray &ray, BVHNode* bvh, Hit &hit, const Scene &scene) {

    bool result = false;
    float t = RayBoxIntersection(ray,bvh->box);
    if(t < 0.0f || t == std::numeric_limits<float>::max()) {
        hit.isHit = false;
        return false;
    }


    if(bvh->faces.size() > 0) {
        Face currentFace = bvh->faces[0];
        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        Ray transformRay = ray;
        Matrix4x4 transform = currentFace.transformationMatrix;
        transform = transform.inverse();
        transformRay.direction = transform.MultiplicationWithVector(transformRay.direction);
        transformRay.origin = transform.MultiplicationWithPoint(transformRay.origin);

        Hit primitiveHit = TriangleIntersection(transformRay,v0,v1,v2,currentFace.v0_id,currentFace.v1_id,currentFace.v2_id,scene);
        primitiveHit.hitPoint = currentFace.transformationMatrix.MultiplicationWithPoint(primitiveHit.hitPoint);
        //primitiveHit.surfaceNormal = transform.inverse().MultiplicationWithVector(primitiveHit.surfaceNormal);
        primitiveHit.surfaceNormal = NormalizeVector3f(transform.transpose().MultiplicationWithVector(primitiveHit.surfaceNormal));

        if (primitiveHit.isHit && primitiveHit.t < hit.t) {
            //std::cout << primitiveHit.hitPoint.x << "****" <<hit.hitPoint.x << "\n";
            hit = primitiveHit;
            hit.materialID = currentFace.materialID;
            if(scene.texCoord_data.size() > 0) {
                Hit normalTextureHit = TriangleTangentAndBitangentVectors(hit,currentFace,scene);
                hit = normalTextureHit;
            }

            for(auto textureid: currentFace.textureIDs) {
                hit.textureIDs.push_back(textureid);
            }
            
            return true;
        }
    }

    if(bvh->left) {
        Hit hitleft;
        hitleft.t = std::numeric_limits<float>::max();
        bool leftresult = BVHIntersection(ray,bvh->left,hitleft,scene);
        if(leftresult && hitleft.t < hit.t && hitleft.t > 0.0f) {
            hit = hitleft;
            hit.isHit = true;
            result = true;
        }
    }
    

    if(bvh->right) {
        Hit hitright;
        hitright.t = std::numeric_limits<float>::max();
        bool rightresult = BVHIntersection(ray,bvh->right,hitright,scene);
        if(rightresult && hitright.t < hit.t && hitright.t > 0.0f) {
            hit = hitright;
            hit.isHit = true;
            result = true;
        }
    }

    return result;
}

std::vector<BoundingBox> computeBoundingBoxesForFaces(std::vector<Face> &allFaces, const Scene &scene) {
    
    std::vector<BoundingBox> bbVector;

    // Loop through the objects in the specified range and expand the bounding box
    for (int i = 0; i < allFaces.size(); i++) {

        // Initialize the bounding box with extreme values
        Vec3f minPoint{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
        Vec3f maxPoint ={-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};

        Face currentFace = allFaces[i];
        Vec3f v0 = scene.vertex_data[currentFace.v0_id - 1];
        Vec3f v1 = scene.vertex_data[currentFace.v1_id - 1];
        Vec3f v2 = scene.vertex_data[currentFace.v2_id - 1];

        minPoint.x = std::min(v0.x,v1.x);
        minPoint.x = std::min(minPoint.x,v2.x);
        minPoint.y = std::min(v0.y,v1.y);
        minPoint.y = std::min(minPoint.y,v2.y);
        minPoint.z = std::min(v0.z,v1.z);
        minPoint.z = std::min(minPoint.z,v2.z);

        maxPoint.x = std::max(v0.x,v1.x);
        maxPoint.x = std::max(maxPoint.x,v2.x);
        maxPoint.y = std::max(v0.y,v1.y);
        maxPoint.y = std::max(maxPoint.y,v2.y);
        maxPoint.z = std::max(v0.z,v1.z);
        maxPoint.z = std::max(maxPoint.z,v2.z);

        float x = (minPoint.x + maxPoint.x) * 0.5f;
        float y = (minPoint.y + maxPoint.y) * 0.5f;
        float z = (minPoint.z + maxPoint.z) * 0.5f;
        Vec3f center{x,y,z};    

        allFaces[i].boundingBox.min = minPoint;
        allFaces[i].boundingBox.max = maxPoint;
        allFaces[i].boundingBox.center = center;

        bbVector.push_back(allFaces[i].boundingBox);
    }

    return bbVector;
}

BVHNode* buildBVH(std::vector<Face> &allFaces, int start, int end, int splitAxis) {
    // check if there's mesh
    if(allFaces.size() == 0){
        return nullptr;
    }

    BVHNode* node = new BVHNode();

    // Compute the bounding box of all objects in this node
    BoundingBox box;
    Vec3f nodeMin{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max()};
    Vec3f nodeMax ={-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max(), -std::numeric_limits<float>::max()};

    for(int i = start; i < end; i++) {
        BoundingBox currentBox = allFaces[i].boundingBox;
        BoundingBox transformedBB = TransformBoundingBox(currentBox,allFaces[i].transformationMatrix);

        nodeMin.x = std::min(transformedBB.min.x,nodeMin.x);
        nodeMin.y = std::min(transformedBB.min.y,nodeMin.y);
        nodeMin.z = std::min(transformedBB.min.z,nodeMin.z);

        nodeMax.x = std::max(transformedBB.max.x,nodeMax.x);
        nodeMax.y = std::max(transformedBB.max.y,nodeMax.y);
        nodeMax.z = std::max(transformedBB.max.z,nodeMax.z);

        
    }
    box.min = nodeMin;
    box.max = nodeMax;
    node->box = box;

    // If there's only one object, make it a leaf node
    if (end - start == 1) {
        node->faces.push_back(allFaces[start]);
        return node;
    }

    // Sort objects by their centroids along one axis (e.g., x-axis)
    // split axis: 0 -> x
    // split axis: 1 -> y
    // split axis: 2 -> z
    if(splitAxis > 2) {splitAxis = 0;}

    if(splitAxis == 0) {
        std::sort(allFaces.begin() + start, allFaces.begin() + end,
              [splitAxis](Face a, Face b) {
                  return a.boundingBox.center.x < b.boundingBox.center.x;
              });
    }
    if(splitAxis == 1) {
        std::sort(allFaces.begin() + start, allFaces.begin() + end,
              [splitAxis](Face a, Face b) {
                  return a.boundingBox.center.y < b.boundingBox.center.y;
              });
    }
    if(splitAxis == 2) {
        std::sort(allFaces.begin() + start, allFaces.begin() + end,
              [splitAxis](Face a, Face b) {
                  return a.boundingBox.center.z < b.boundingBox.center.z;
              });
    }
    

    // Split objects into two groups
    int mid = (start+end) / 2;

    // Recursively build the left and right child nodes
    node->left = buildBVH(allFaces, start, mid, splitAxis+1);
    node->right = buildBVH(allFaces, mid, end, splitAxis+1);
    return node;
}


//Textures
Vec3f fetchImage(int i, int j, int width, int height , unsigned char* data, Texture& texture) {
    
    i = i % width;
    j = j % height;
    Vec3f fetchingColor{0,0,0};
    

    if(texture.isHDRTexture) {
        int pixel_index = (j * width + i) * 4;
        fetchingColor.x = texture.float_data[pixel_index];
        fetchingColor.y = texture.float_data[++pixel_index];
        fetchingColor.z = texture.float_data[++pixel_index];
    }
    else {
        int pixel_index = (j * width + i) * 3;
        fetchingColor.x = data[pixel_index];
        fetchingColor.y = data[++pixel_index];
        fetchingColor.z = data[++pixel_index];
    }

    
    return fetchingColor;
}

Vec3f calculateTextureColor(Vec2f& uvCoords, Texture& texture, const Scene& scene) {

    Vec3f textureColor{0,0,0};

    if(texture.interpolation == 0) {
        //bilinear
        float i = uvCoords.u * texture.width;
        float j = uvCoords.v * texture.height;
        float p = floor(i);
        float q = floor(j);
        float dx = i-p;
        float dy = j-q;        
        
        Vec3f a = fetchImage(p,q,texture.width,texture.height,texture.data,texture);
        Vec3f b = fetchImage(p+1,q,texture.width,texture.height,texture.data,texture);
        Vec3f c = fetchImage(p,q+1,texture.width,texture.height,texture.data,texture);
        Vec3f d = fetchImage(p+1,q+1,texture.width,texture.height,texture.data,texture);
        textureColor.x = a.x * (1-dx) * (1-dy) +
                         b.x * dx * (1-dy) +
                         c.x * (1-dx) * dy +
                         d.x * dx * dy;
        
        textureColor.y = a.y * (1-dx) * (1-dy) +
                         b.y * dx * (1-dy) +
                         c.y * (1-dx) * dy +
                         d.y * dx * dy;

        textureColor.z = a.z * (1-dx) * (1-dy) +
                         b.z * dx * (1-dy) +
                         c.z * (1-dx) * dy +
                         d.z * dx * dy;
    }
    else if(texture.interpolation == 1) {
        //nearest
        float i = uvCoords.u * texture.width;
        float j = uvCoords.v * texture.height;
        
        textureColor = fetchImage(round(i),round(j),texture.width,texture.height,texture.data,texture);
    }

    return textureColor;
}

Vec3f GenerateRandomGradient(int x, int y, int z) {
    Vec3f gradients[12] = {
        {-1, -1, 0}, {-1, 1, 0}, {1, -1, 0}, {1, 1, 0},
        {-1, 0, -1}, {1, 0, -1}, {-1, 0, 1}, {1, 0, 1},
        {0, -1, -1}, {0, 1, -1}, {0, -1, 1}, {0, 1, 1}
    };

    // Create a pseudo-random index using a hash function
    int seed = x * 73856093 ^ y * 19349663 ^ z * 83492791; 
    seed = (seed << 13) ^ seed;
    int randomIndex = (seed & 0x7FFFFFFF) % 12;

    return gradients[randomIndex];
}

float ContributionFunctionPerlin(float x) {
    float absX = abs(x);
    float y = -6 * pow(absX, 5) + 15 * pow(absX, 4) - 10 * pow(absX, 3) + 1;
    
    if(absX < 1) {
        return y;
    }
    else {
        return 0;
    }
}



Vec3f PerlinNoiseForPoint(Vec3f& pointForPerlin, int conversion, int noiseScale) {

    Vec3f p = MultiplyVectorWithConstant(pointForPerlin,noiseScale);

    float a = floor(p.x);
    float b = floor(p.y);
    float c = floor(p.z);

    // generate 8 random vectors in [-1,1] 
    Vec3f g0 = GenerateRandomGradient(a,b,c);
    Vec3f g1 = GenerateRandomGradient(a+1,b,c);
    Vec3f g2 = GenerateRandomGradient(a+1,b+1,c);
    Vec3f g3 = GenerateRandomGradient(a+1,b,c+1);
    Vec3f g4 = GenerateRandomGradient(a+1,b+1,c+1);
    Vec3f g5 = GenerateRandomGradient(a,b+1,c);
    Vec3f g6 = GenerateRandomGradient(a,b+1,c+1);
    Vec3f g7 = GenerateRandomGradient(a,b,c+1);



    Vec3f p0{a,b,c};
    Vec3f p1{a+1,b,c};
    Vec3f p2{a+1,b+1,c};
    Vec3f p3{a+1,b,c+1};
    Vec3f p4{a+1,b+1,c+1};
    Vec3f p5{a,b+1,c};
    Vec3f p6{a,b+1,c+1};
    Vec3f p7{a,b,c+1};

    // vectors from corners to point
    Vec3f v0,v1,v2,v3,v4,v5,v6,v7;
    v0.x = p.x - p0.x;
    v0.y = p.y - p0.y;
    v0.z = p.z - p0.z;

    v1.x = p.x - p1.x;
    v1.y = p.y - p1.y;
    v1.z = p.z - p1.z;

    v2.x = p.x - p2.x;
    v2.y = p.y - p2.y;
    v2.z = p.z - p2.z;

    v3.x = p.x - p3.x;
    v3.y = p.y - p3.y;
    v3.z = p.z - p3.z;

    v4.x = p.x - p4.x;
    v4.y = p.y - p4.y;
    v4.z = p.z - p4.z;

    v5.x = p.x - p5.x;
    v5.y = p.y - p5.y;
    v5.z = p.z - p5.z;

    v6.x = p.x - p6.x;
    v6.y = p.y - p6.y;
    v6.z = p.z - p6.z;

    v7.x = p.x - p7.x;
    v7.y = p.y - p7.y;
    v7.z = p.z - p7.z;

    // dot product of each corner
    float d0 = DotProduct(v0,g0);
    float d1 = DotProduct(v1,g1);
    float d2 = DotProduct(v2,g2);
    float d3 = DotProduct(v3,g3);
    float d4 = DotProduct(v4,g4);
    float d5 = DotProduct(v5,g5);
    float d6 = DotProduct(v6,g6);
    float d7 = DotProduct(v7,g7);

    // contribution of each corner
    float w0 = ContributionFunctionPerlin(p.x - p0.x) * ContributionFunctionPerlin(p.y - p0.y) * ContributionFunctionPerlin(p.z - p0.z);
    float w1 = ContributionFunctionPerlin(p.x - p1.x) * ContributionFunctionPerlin(p.y - p1.y) * ContributionFunctionPerlin(p.z - p1.z);
    float w2 = ContributionFunctionPerlin(p.x - p2.x) * ContributionFunctionPerlin(p.y - p2.y) * ContributionFunctionPerlin(p.z - p2.z);
    float w3 = ContributionFunctionPerlin(p.x - p3.x) * ContributionFunctionPerlin(p.y - p3.y) * ContributionFunctionPerlin(p.z - p3.z);
    float w4 = ContributionFunctionPerlin(p.x - p4.x) * ContributionFunctionPerlin(p.y - p4.y) * ContributionFunctionPerlin(p.z - p4.z);
    float w5 = ContributionFunctionPerlin(p.x - p5.x) * ContributionFunctionPerlin(p.y - p5.y) * ContributionFunctionPerlin(p.z - p5.z);
    float w6 = ContributionFunctionPerlin(p.x - p6.x) * ContributionFunctionPerlin(p.y - p6.y) * ContributionFunctionPerlin(p.z - p6.z);
    float w7 = ContributionFunctionPerlin(p.x - p7.x) * ContributionFunctionPerlin(p.y - p7.y) * ContributionFunctionPerlin(p.z - p7.z);

    float noisePrime = w0*d0 + w1*d1 + w2*d2 + w3*d3 + w4*d4 + w5*d5 + w6*d6 + w7*d7;

    Vec3f perlinNoise;
    if(conversion == 0) {
        float noise = abs(noisePrime);
        perlinNoise.x = noise;
        perlinNoise.y = noise;
        perlinNoise.z = noise;
    }
    else if(conversion == 1) {
        float noise = (noisePrime + 1) / 2;
        perlinNoise.x = noise;
        perlinNoise.y = noise;
        perlinNoise.z = noise;
    }

    return perlinNoise;
}

Hit TriangleTangentAndBitangentVectors(Hit& hit, Face& face, const Scene& scene) {

    Vec3f edge1 = SubstractVectors(scene.vertex_data[face.v1_id - 1],scene.vertex_data[face.v0_id - 1]);
    Vec3f edge2 = SubstractVectors(scene.vertex_data[face.v2_id - 1],scene.vertex_data[face.v1_id - 1]);

    edge1 = NormalizeVector3f(edge1);
    edge2 = NormalizeVector3f(edge2);

    Vec2f v0UV = scene.texCoord_data[face.v0_id - 1];
    v0UV.u = floor(v0UV.u);
    v0UV.v = floor(v0UV.v);
    Vec2f v1UV = scene.texCoord_data[face.v1_id - 1];
    v1UV.u = floor(v1UV.u);
    v1UV.v = floor(v1UV.v);
    Vec2f v2UV = scene.texCoord_data[face.v2_id - 1];
    v2UV.u = floor(v2UV.u);
    v2UV.v = floor(v2UV.v);

    Vec2f deltaUV1;
    deltaUV1.u = v1UV.u - v0UV.u;
    deltaUV1.v = v1UV.v - v0UV.v;
    
    Vec2f deltaUV2;
    deltaUV2.u = v2UV.u - v1UV.u;
    deltaUV2.v = v2UV.v - v1UV.v;

    float determinant = deltaUV1.u * deltaUV2.v - deltaUV1.v * deltaUV2.u;

    Vec3f tangent = (SubstractVectors(MultiplyVectorWithConstant(edge1,deltaUV2.v),MultiplyVectorWithConstant(edge2,deltaUV1.v)));
    tangent.x = tangent.x / determinant;
    tangent.y = tangent.y / determinant;
    tangent.z = tangent.z / determinant;

    Vec3f bitangent = (SubstractVectors(MultiplyVectorWithConstant(edge2,deltaUV1.u),MultiplyVectorWithConstant(edge1,deltaUV2.u)));
    bitangent.x = bitangent.x / determinant;
    bitangent.y = bitangent.y / determinant;
    bitangent.z = bitangent.z / determinant;

    Hit newHit = hit;
    newHit.tangentVector = NormalizeVector3f(tangent);
    newHit.bitangentVector = NormalizeVector3f(bitangent);

    return newHit;
}

Hit NormalMapping(Hit& hit, Texture& texture, const Scene& scene) {

    Vec3f texColor = calculateTextureColor(hit.uvCoordinates,texture,scene);
    Vec3f directions;

    directions.x = (texColor.x / 127.5f) - 1;
    directions.y = (texColor.y / 127.5f) - 1;
    directions.z = (texColor.z / 127.5f) - 1;

    directions = NormalizeVector3f(directions);

    Vec3f normalTangent = NormalizeVector3f(hit.tangentVector);
    Vec3f normalBitangent = NormalizeVector3f(hit.bitangentVector);

    // Final transformation
    Vec3f newNormal;
    newNormal.x = (normalTangent.x * directions.x) + (normalBitangent.x * directions.y) + (hit.surfaceNormal.x * directions.z);
    newNormal.y = (normalTangent.y * directions.x) + (normalBitangent.y * directions.y) + (hit.surfaceNormal.y * directions.z);
    newNormal.z = (normalTangent.z * directions.x) + (normalBitangent.z * directions.y) + (hit.surfaceNormal.z * directions.z);

    // Normalize the new normal
    Hit newHit = hit;
    newHit.surfaceNormal = NormalizeVector3f(newNormal);

    return newHit;
}

Vec3f BumpMapping(Hit& hit, Texture& texture) {

    Vec3f dpDU = hit.tangentVector;
    Vec3f dpDV = hit.bitangentVector;

    float i = round(hit.uvCoordinates.u * texture.width) ;
    float j = round(hit.uvCoordinates.v * texture.height);
    Vec3f dhDU = MultiplyVectorWithConstant(SubstractVectors(fetchImage(i+1,j,texture.width,texture.height,texture.data,texture), fetchImage(i,j,texture.width,texture.height,texture.data,texture)),texture.bumpFactor);  
    Vec3f dhDV = MultiplyVectorWithConstant(SubstractVectors(fetchImage(i,j+1,texture.width,texture.height,texture.data,texture), fetchImage(i,j,texture.width,texture.height,texture.data,texture)),texture.bumpFactor);  

    float averageColorU = (dhDU.x + dhDU.y + dhDU.z) / 3;
    float averageColorV = (dhDV.x + dhDV.y + dhDV.z) / 3;


    Vec3f dqDU = NormalizeVector3f(SumVectors(dpDU,MultiplyVectorWithConstant(hit.surfaceNormal,averageColorU)));
    Vec3f dqDV = NormalizeVector3f(SumVectors(dpDV,MultiplyVectorWithConstant(hit.surfaceNormal,averageColorV)));

    Vec3f newNormal = NormalizeVector3f(CrossProduct(dqDV,dqDU));

    if(DotProduct(newNormal,hit.surfaceNormal) < 0 ) {
        newNormal = MultiplyVectorWithConstant(newNormal,-1);
    }

    return newNormal;
}


bool saveExr(const float* floatImage, int width, int height, const std::string &filename)
{
    std::vector<float> rChannel(width * height);
    std::vector<float> gChannel(width * height);
    std::vector<float> bChannel(width * height);

    for (int i = 0; i < width * height; i++) {
        float r = floatImage[3*i + 0]; // R
        float g = floatImage[3*i + 1]; // G
        float b = floatImage[3*i + 2]; // B

        rChannel[i] = r;
        gChannel[i] = g;
        bChannel[i] = b;
    }

    EXRImage exrImage;
    InitEXRImage(&exrImage);
    exrImage.num_channels = 3;
    exrImage.width = width;
    exrImage.height = height;

    // TinyEXR BGR 
    std::vector<float>* imagePtr[3];
    imagePtr[0] = &bChannel;
    imagePtr[1] = &gChannel;
    imagePtr[2] = &rChannel;

    exrImage.images = new unsigned char*[3];
    exrImage.images[0] = (unsigned char*)imagePtr[0]->data(); // B
    exrImage.images[1] = (unsigned char*)imagePtr[1]->data(); // G
    exrImage.images[2] = (unsigned char*)imagePtr[2]->data(); // R

    EXRHeader exrHeader;
    InitEXRHeader(&exrHeader);
    exrHeader.num_channels = 3;
    exrHeader.channels = (EXRChannelInfo*)malloc(sizeof(EXRChannelInfo)*3);

    strcpy(exrHeader.channels[0].name, "B");
    strcpy(exrHeader.channels[1].name, "G");
    strcpy(exrHeader.channels[2].name, "R");

    exrHeader.pixel_types = (int*)malloc(sizeof(int)*3);
    exrHeader.requested_pixel_types = (int*)malloc(sizeof(int)*3);
    for (int i = 0; i < 3; i++) {
        exrHeader.pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT;
        exrHeader.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT; 
    }

    const char* err = nullptr;
    int ret = SaveEXRImageToFile(&exrImage, &exrHeader, filename.c_str(), &err);

    delete[] exrImage.images;
    free(exrHeader.channels);
    free(exrHeader.pixel_types);
    free(exrHeader.requested_pixel_types);

    if (ret != TINYEXR_SUCCESS) {
        if (err) {
            fprintf(stderr, "Save EXR error: %s\n", err);
            FreeEXRErrorMessage(err);
        }
        return false;
    }
    return true;
}

std::string GetFileExtension(const std::string &filename)
{
    size_t dotPos = filename.find_last_of('.');
    if (dotPos == std::string::npos) {
        return "";
    }

    std::string ext = filename.substr(dotPos + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    return ext;
}


void ToneMappingForImage(unsigned char* toneMappedLDR, float* imageHDR, int width, int height, ToneMapping toneMap){

    std::vector<float> luminances;

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {

            int pixel_index = (i * width + j) * 3;

            float red = imageHDR[pixel_index] * 0.2126f;
            float green = imageHDR[pixel_index + 1] * 0.7152f;
            float blue = imageHDR[pixel_index + 2] * 0.0722f;

            float y_i = red + green + blue;
            luminances.push_back(y_i);

        }
    }


    float sumLuminances = 0;
    for(int i = 0; i < width*height; i++) {
        sumLuminances += log(0.00001f + luminances[i]);
    }

    float luminanceMean = exp(sumLuminances/(width*height));
    std::vector<float> zoneLuminances;

    for(int i = 0; i < width*height; i++) {
        float zoneLuminance = (toneMap.keyValue/luminanceMean) * luminances[i];
        zoneLuminances.push_back(zoneLuminance);
    }


    // find Lwhite NOTE: BURNPERCENT==0 CASE MIGHT BE CHANGED
    float Lwhite;
    std::vector<float> sortedLuminances(zoneLuminances);
    std::sort(sortedLuminances.begin(), sortedLuminances.end());
    if(toneMap.burnPercent == 0) {
        // just to assign a value
        Lwhite = sortedLuminances.back();
    }
    else {
        float percentile = 100-toneMap.burnPercent;
        int whiteIndex = round((percentile * sortedLuminances.size()) / 100) - 1;
        Lwhite = sortedLuminances[whiteIndex];
    }
	

    std::vector<float> compressedLuminances;
    if(toneMap.burnPercent == 0) {
        for(int i = 0; i < width*height; i++) {
            float compressedValue = (zoneLuminances[i]) / (1 + zoneLuminances[i]);
            compressedLuminances.push_back(compressedValue);
        }
    }
    else {
        for(int i = 0; i < width*height; i++) {
            float compressedValue = (zoneLuminances[i] * ( 1 + (zoneLuminances[i] / pow(Lwhite,2)))) / (1 + zoneLuminances[i]);
            compressedLuminances.push_back(compressedValue);
        }
    }
    

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {

            int pixel_index = (i * width + j) * 3;
            int lum_index = (i * width + j);

            float red = imageHDR[pixel_index];
            float green = imageHDR[pixel_index + 1];
            float blue = imageHDR[pixel_index + 2];

            float y_i = red * 0.2126f + green * 0.7152f + blue * 0.0722f;
            
            float r_o;
            float g_o;
            float b_o;

            if (y_i < 1e-8f) {
                // If the luminance is effectively zero, set color to 0
                r_o = 0.0f;
                g_o = 0.0f;
                b_o = 0.0f;
            } else {
                r_o = compressedLuminances[lum_index] * powf((red   / y_i), toneMap.saturation);
                g_o = compressedLuminances[lum_index] * powf((green / y_i), toneMap.saturation);
                b_o = compressedLuminances[lum_index] * powf((blue  / y_i), toneMap.saturation);
            }

            float r_f = 255 * pow(r_o,(1/toneMap.gamma));
            float g_f = 255 * pow(g_o,(1/toneMap.gamma));
            float b_f = 255 * pow(b_o,(1/toneMap.gamma));

            toneMappedLDR[pixel_index] = round(findMin(r_f,255));
            toneMappedLDR[pixel_index + 1] = round(findMin(g_f,255));
            toneMappedLDR[pixel_index + 2] = round(findMin(b_f,255));

        }
    }

}


// attenuation fix
// store triangle normals 
// diğer objeleri de bvh içine al

void raytracer_render(int start, int end, int width, int height, unsigned char* image, float* floatImage, Camera& camera, const parser::Scene& scene, std::vector<BVHNode*> bvh) {
    
    // for random number generation
    std::random_device rd;                        
    std::mt19937 mt(rd());                        
    std::uniform_real_distribution<float> dist(0.0f, 1.0f); // Uniform distribution [0, 1)

    int numOfRowsAndColumns = static_cast<int>(sqrt(camera.numSamples));

    for (int y = start; y < end; ++y) {
        for (int x = 0; x < width; ++x) {

            int pixel_index = (y * width + x) * 3;

            Vec3f accumulatedColor = {0,0,0};

            // subpixels
            for(int i = 0; i < numOfRowsAndColumns; i++) {
                for(int j = 0; j < numOfRowsAndColumns; j++) {

                    // random location
                    float randomX = dist(mt);
                    float randomY = dist(mt);

                    // subpixel coordinates
                    float subpixelX;
                    float subpixelY;
                    if(camera.numSamples == 1) {
                        subpixelX = (i + 0.5f) / numOfRowsAndColumns;
                        subpixelY = (j + 0.5f) / numOfRowsAndColumns;
                    }
                    else {
                        subpixelX = (i + randomX) / numOfRowsAndColumns;
                        subpixelY = (j + randomY) / numOfRowsAndColumns;
                    }
                    
                    
                    // Send ray through the jittered subpixel
                    Ray currentRay = SendRayToPixel(camera, x, y, subpixelX, subpixelY, mt, dist);
                    currentRay.reflectionDepth = 0;
                    currentRay.time = dist(mt);

                    Hit closestHit;
                    closestHit.isHit = false;
                    closestHit.t = std::numeric_limits<float>::max();

                    closestHit = FindClosestRayIntersection(currentRay, scene, bvh);
                    
                    Vec3f color = {0,0,0};
            
                    if (closestHit.isHit) {

                        if(closestHit.isLightHit) {
                            color = closestHit.color;
                        }
                        else {
                            color = ApplyShadings(currentRay, closestHit, scene,bvh,mt,dist);
                            //clampColor(color);
                        }
                        
                    } 
                    else {

                        if(scene.hasBackgroundTexture) {
                            Texture backGround = scene.backGroundTexture;
                            Vec2f uvCoords;
                            float a = x % backGround.width;
                            float b = y % backGround.height; 
                            uvCoords.u = a / backGround.width;
                            uvCoords.v = b / backGround.height; 

                            color = calculateTextureColor(uvCoords,backGround,scene);
                        }
                        else if(scene.sphericalDirectional_lights.size() > 0) {
                            SphericalDirectionalLight currentSphericalDirectionalLight = scene.sphericalDirectional_lights[0];
                            Vec3f vectorForCalculation = currentRay.direction;
                            float u,v;
                            if(currentSphericalDirectionalLight.type == 0) {
                                // latlong
                                u = ((1 + ( ( atan2(vectorForCalculation.x,-vectorForCalculation.z) / M_PI ) ) ) / 2);
                                v = acos(vectorForCalculation.y) / M_PI;
                            }
                            else if(currentSphericalDirectionalLight.type == 1) {
                                // probe
                                float r = (1/M_PI) * ( acos(-vectorForCalculation.z) / sqrtf(pow(vectorForCalculation.x,2) + pow(vectorForCalculation.y,2)));
                                u = (r*vectorForCalculation.x + 1) / 2;
                                v = (-r*vectorForCalculation.y + 1) / 2;
                            }


                            ImageWithData currentImage = scene.imageDataVector[currentSphericalDirectionalLight.imageID - 1];
                            int pixelI = round(u * currentImage.width);
                            int pixelJ = round(v * currentImage.height);

                            pixelI = pixelI % currentImage.width;
                            pixelJ = pixelJ % currentImage.height;
                            Vec3f fetchingColor{0,0,0};
                            int pixel_index = (pixelJ * currentImage.width + pixelI) * 4;

                            fetchingColor.x = currentImage.float_data[pixel_index];
                            fetchingColor.y = currentImage.float_data[++pixel_index];
                            fetchingColor.z = currentImage.float_data[++pixel_index];

                            fetchingColor.x = 2 * M_PI * fetchingColor.x;
                            fetchingColor.y = 2 * M_PI * fetchingColor.y;
                            fetchingColor.z = 2 * M_PI * fetchingColor.z;

                            color.x += fetchingColor.x;
                            color.y += fetchingColor.y;
                            color.z += fetchingColor.z;

                        }
                        else {
                            color.x = scene.background_color.x;
                            color.y = scene.background_color.y;
                            color.z = scene.background_color.z;
                        }
                    }

                    accumulatedColor.x += color.x;
                    accumulatedColor.y += color.y;
                    accumulatedColor.z += color.z;
                }
            }

            // Average accumulated color
            accumulatedColor.x /= static_cast<float>(camera.numSamples);
            accumulatedColor.y /= static_cast<float>(camera.numSamples);
            accumulatedColor.z /= static_cast<float>(camera.numSamples);

            // values for HDR Image
            floatImage[pixel_index]   = accumulatedColor.x; // R
            floatImage[pixel_index + 1] = accumulatedColor.y; // G
            floatImage[pixel_index + 2] = accumulatedColor.z; // B

            // clamping for LDR Image
            clampColor(accumulatedColor);

            // rounding for LDR Image
            image[pixel_index] = round(accumulatedColor.x);  // R
            image[pixel_index + 1] = round(accumulatedColor.y);  // G
            image[pixel_index + 2] = round(accumulatedColor.z);  // B

            
        }
    }
}

int main(int argc, char* argv[])
{
    parser::Scene scene;

    scene.loadFromXml(argv[1]);
    
    int width, height;
    unsigned char* image;
    float* floatImage;
    unsigned char* toneMappedImage;


    //std::vector<BoundingBox> bbVector = computeBoundingBoxesForFaces(scene.allMeshFaces,scene);
    //BVHNode* bvhTree = buildBVH(scene.allMeshFaces,0,scene.allMeshFaces.size(),0);


    std::vector<BVHNode *> bvhTrees;
    for(int i = 0; i < scene.meshes.size(); i++) {
        std::vector<BoundingBox> boundingBoxForMesh = computeBoundingBoxesForFaces(scene.meshes[i].faces,scene);
        BVHNode* bvhTreeForMesh = buildBVH(scene.meshes[i].faces,0,scene.meshes[i].faces.size(),0);
        bvhTrees.push_back(bvhTreeForMesh);
    }

    for(int i = 0; i < scene.imageDataVector.size(); i++) {
        std::string ext = GetFileExtension(scene.imageDataVector[i].imageName.c_str());

        if(ext == "exr") {
            float* data_float;
            data_float = nullptr;
            const char* err = nullptr;
            int ret = LoadEXR(&data_float, &scene.imageDataVector[i].width, &scene.imageDataVector[i].height, scene.imageDataVector[i].imageName.c_str(), &err);
            if (ret != TINYEXR_SUCCESS) {
                if (err) {
                    std::cerr << "Failed to load EXR: " << err << "\n";
                    FreeEXRErrorMessage(err);
                }
                return false;
            }

            scene.imageDataVector[i].float_data = data_float;
            scene.imageDataVector[i].channels = 3;

            //free(data_float);
            std:: cout << scene.imageDataVector[i].imageName.c_str() << " image is read.\n";

        }
        else {
            // png,jpg,jpeg images
            scene.imageDataVector[i].data = stbi_load(scene.imageDataVector[i].imageName.c_str(), &scene.imageDataVector[i].width, &scene.imageDataVector[i].height, &scene.imageDataVector[i].channels, 3);
            std:: cout << scene.imageDataVector[i].imageName.c_str() << " image is read.\n";
        }
    }


    if(scene.hasBackgroundTexture) {
        scene.backGroundTexture.data = scene.imageDataVector[scene.backGroundTexture.imageId - 1].data;
    }

    for(int i = 0; i < scene.textures.size(); i++) {

        if(scene.textures[i].type == 0) {
            std::string ext = GetFileExtension(scene.imageDataVector[scene.textures[i].imageId - 1].imageName.c_str());
            if(ext == "exr") {
                scene.textures[i].isHDRTexture = true;
                scene.textures[i].float_data = scene.imageDataVector[scene.textures[i].imageId - 1].float_data;
                scene.textures[i].width = scene.imageDataVector[scene.textures[i].imageId - 1].width;
                scene.textures[i].height = scene.imageDataVector[scene.textures[i].imageId - 1].height;
                scene.textures[i].channels = scene.imageDataVector[scene.textures[i].imageId - 1].channels;

            }
            else {
                scene.textures[i].data = scene.imageDataVector[scene.textures[i].imageId - 1].data;
                scene.textures[i].width = scene.imageDataVector[scene.textures[i].imageId - 1].width;
                scene.textures[i].height = scene.imageDataVector[scene.textures[i].imageId - 1].height;
                scene.textures[i].channels = scene.imageDataVector[scene.textures[i].imageId - 1].channels;
            }

            
        }
    }

    clock_t begin = clock();

    for(int camIterator = 0; camIterator < scene.cameras.size(); camIterator++) {

        Camera currentCam = scene.cameras[camIterator];

        // normalize vectors
        currentCam.gaze = NormalizeVector3f(currentCam.gaze);
        currentCam.up = NormalizeVector3f(currentCam.up);

        // u = v x w
        Vec3f u = NormalizeVector3f(CrossProduct(currentCam.up,NegateVector(currentCam.gaze)));

        // orthonormal for up
        currentCam.up.x = NormalizeVector3f(CrossProduct(NegateVector(currentCam.gaze),u)).x;
        currentCam.up.y = NormalizeVector3f(CrossProduct(NegateVector(currentCam.gaze),u)).y;
        currentCam.up.z = NormalizeVector3f(CrossProduct(NegateVector(currentCam.gaze),u)).z;


        width = scene.cameras[camIterator].image_width;
        height = scene.cameras[camIterator].image_height;

        image = new unsigned char [width * height * 3];
        floatImage = new float[width * height * 3];
        toneMappedImage = new unsigned char [width * height * 3];


        const int num_threads = std::thread::hardware_concurrency();
        std::vector<std::thread> threads;
        int rows_per_thread = height / num_threads;

        for (int i = 0; i < num_threads; ++i) {
            int start_y = i * rows_per_thread;
            int end_y = (i == num_threads - 1) ? height : start_y + rows_per_thread;

            threads.push_back(std::thread(raytracer_render, start_y, end_y, width, height, image, floatImage, std::ref(currentCam), std::ref(scene), std::ref(bvhTrees)));
        }

        for (auto& thread : threads) {
            thread.join();
        }


        if (scene.sceneToneMap.hasToneMapping) {
            std::string exrOutput = currentCam.image_name;
            bool success = saveExr(floatImage, width, height, exrOutput);
            if (!success) {
                std::cerr << "Failed to save EXR image!\n";
            }
            
            ToneMappingForImage(toneMappedImage,floatImage,width,height,scene.sceneToneMap);
            
            std::string pngOutput = exrOutput.substr(0, exrOutput.find_last_of('.')) + ".png";
            stbi_write_png(pngOutput.c_str(), width, height, 3, toneMappedImage, width * 3);
        }
        else {
            std::string output_file = currentCam.image_name;
            stbi_write_png(output_file.c_str(), width, height, 3, image, width * 3);
        }


        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Time elapsed: " << elapsed_secs << " seconds\n";

        

        delete[] image; 
    }

}
