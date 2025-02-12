#ifndef __HW1__PARSER__
#define __HW1__PARSER__

#include <string>
#include <vector>
#include <cmath>


namespace parser
{

    //Notice that all the structures are as simple as possible
    //so that you are not enforced to adopt any style or design.
    struct Vec3f
    {
        float x, y, z;
    };

    struct Vec3i
    {
        int x, y, z;
    };

    struct Vec4f
    {
        float x, y, z, w;
    };

    struct Vec2f
    {
        float u,v;
    };

    class BoundingBox {
    
        public:
            Vec3f min;
            Vec3f max;
            Vec3f center;

            BoundingBox() {
                Vec3f maxNew{};
                this->max.x = maxNew.x;
                this->max.y = maxNew.y;
                this->max.z = maxNew.z;

                Vec3f minNew{};
                this->min.x = minNew.x;
                this->min.y = minNew.y;
                this->min.z = minNew.z;
            }

            BoundingBox(const Vec3f& minVec, const Vec3f& maxVec) {this->min = minVec; this->max = maxVec;}


    };

    class Matrix4x4
    {
        public:
            float elements[4][4];

            Matrix4x4() {
                for(int i = 0; i < 4; i++) {
                    for(int j = 0; j < 4; j++) {
                        this->elements[i][j] = 0.0f;
                    }
                }
            }

            float determinant() {
                float det = 0.0f;
                for (int i = 0; i < 4; ++i) {
                    det += (i % 2 == 0 ? 1 : -1) * elements[0][i] * minorDeterminant(0, i);
                }
                return det;
            }

            float minorDeterminant(int row, int col) {
                float minor[3][3];
                int minorRow = 0, minorCol = 0;

                for (int i = 0; i < 4; ++i) {
                    if (i == row) continue;
                    minorCol = 0;
                    for (int j = 0; j < 4; ++j) {
                        if (j == col) continue;
                        minor[minorRow][minorCol] = elements[i][j];
                        ++minorCol;
                    }
                    ++minorRow;
                }

                return minor[0][0] * (minor[1][1] * minor[2][2] - minor[1][2] * minor[2][1])
                    - minor[0][1] * (minor[1][0] * minor[2][2] - minor[1][2] * minor[2][0])
                    + minor[0][2] * (minor[1][0] * minor[2][1] - minor[1][1] * minor[2][0]);
            }

            Matrix4x4 transpose() {
                Matrix4x4 transposedMatrix;

                for(int i = 0; i < 4; i++) {
                    for(int j = 0; j < 4; j++) {
                        transposedMatrix.elements[i][j] = elements[j][i];
                    }
                }

                return transposedMatrix;
            }

            Matrix4x4 inverse() {
                float det = determinant();

                Matrix4x4 cofactorMatrix;
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        cofactorMatrix.elements[i][j] = ((i + j) % 2 == 0 ? 1 : -1) * minorDeterminant(i, j);
                    }
                }

                Matrix4x4 adjugateMatrix = cofactorMatrix.transpose();
                Matrix4x4 inverseMatrix;
                float invDet = 1.0f / det;

                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        inverseMatrix.elements[i][j] = adjugateMatrix.elements[i][j] * invDet;
                    }
                }

                return inverseMatrix;
            }

            Matrix4x4 multiply(const Matrix4x4& other) {
                Matrix4x4 result;
                
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        result.elements[i][j] = 0.0f;
                        for (int k = 0; k < 4; ++k) {
                            result.elements[i][j] += other.elements[i][k] * elements[k][j];
                        }
                    }
                }

                return result;
            }

            Matrix4x4 TranslationMatrix(float tx, float ty, float tz) {
                Matrix4x4 translation = Matrix4x4();
                translation.elements[0][0] = 1;
                translation.elements[1][1] = 1;
                translation.elements[2][2] = 1;
                translation.elements[3][3] = 1;

                translation.elements[0][3] = tx;
                translation.elements[1][3] = ty;
                translation.elements[2][3] = tz;

                return translation;
            }

            Matrix4x4 ScalingMatrix(float sx, float sy, float sz) {
                Matrix4x4 scaling = Matrix4x4();
                scaling.elements[0][0] = sx;
                scaling.elements[1][1] = sy;
                scaling.elements[2][2] = sz;
                scaling.elements[3][3] = 1;

                return scaling;
            }

            Matrix4x4 RotationMatrix(float theta, float x, float y, float z) {
                Matrix4x4 rotation = Matrix4x4();
                theta = theta * 3.14f / 180.0f;

                if(x == 1) {
                    rotation.elements[0][0] = 1;
                    rotation.elements[3][3] = 1;

                    rotation.elements[1][1] = cos(theta);
                    rotation.elements[1][2] = -sin(theta);
                    rotation.elements[2][1] = sin(theta);
                    rotation.elements[2][2] = cos(theta);
                }
                else if(y == 1) {
                    rotation.elements[1][1] = 1;
                    rotation.elements[3][3] = 1;

                    rotation.elements[0][0] = cos(theta);
                    rotation.elements[0][2] = sin(theta);
                    rotation.elements[2][0] = -sin(theta);
                    rotation.elements[2][2] = cos(theta);
                }
                else if(z == 1) {
                    rotation.elements[2][2] = 1;
                    rotation.elements[3][3] = 1;

                    rotation.elements[0][0] = cos(theta);
                    rotation.elements[0][1] = -sin(theta);
                    rotation.elements[1][0] = sin(theta);
                    rotation.elements[1][1] = cos(theta);
                }

                return rotation;
            }

            Vec3f MultiplicationWithPoint(const Vec3f &p) {
                Vec4f newPoint;
                newPoint.x = p.x;
                newPoint.y = p.y;
                newPoint.z = p.z;
                newPoint.w = 1;

                Vec4f sum{0,0,0,0};

                sum.x += elements[0][0] * newPoint.x;
                sum.x += elements[0][1] * newPoint.y;
                sum.x += elements[0][2] * newPoint.z;
                sum.x += elements[0][3] * newPoint.w;

                sum.y += elements[1][0] * newPoint.x;
                sum.y += elements[1][1] * newPoint.y;
                sum.y += elements[1][2] * newPoint.z;
                sum.y += elements[1][3] * newPoint.w;

                sum.z += elements[2][0] * newPoint.x;
                sum.z += elements[2][1] * newPoint.y;
                sum.z += elements[2][2] * newPoint.z;
                sum.z += elements[2][3] * newPoint.w;
                
                sum.w += elements[3][0] * newPoint.x;
                sum.w += elements[3][1] * newPoint.y;
                sum.w += elements[3][2] * newPoint.z;
                sum.w += elements[3][3] * newPoint.w;

                Vec3f result;
                result.x = sum.x;
                result.y = sum.y;
                result.z = sum.z;
                
                return result;
            }

            Vec3f MultiplicationWithVector(const Vec3f &p) {
                float newPoint[4] = { p.x, p.y, p.z, 0.0f };
                float sum[4];

                for (int i = 0; i < 4; i++)
                {
                    sum[i] = 0.0f;
                    for (int j = 0; j < 4; j++)
                    {
                        sum[i] += elements[i][j] * newPoint[j];
                    }
                }

                Vec3f result;
                result.x = sum[0];
                result.y = sum[1];
                result.z = sum[2];
                return result;
            }

            Matrix4x4 transpose() const {
                Matrix4x4 transposedMatrix;
                
                for (int i = 0; i < 4; ++i) {
                    for (int j = 0; j < 4; ++j) {
                        transposedMatrix.elements[i][j] = elements[j][i];
                    }
                }
                
                return transposedMatrix;
            }
        
    };

    struct Camera
    {
        Vec3f position;
        Vec3f gaze;
        Vec3f up;
        Vec4f near_plane;
        float near_distance;
        int image_width, image_height;
        std::string image_name;
        float fovy;
        int numSamples;
        bool hasDOF;
        float focusDistance;
        float apertureSize;
    };

    struct ToneMapping
    {
        bool hasToneMapping;
        float keyValue;
        float burnPercent;
        float saturation;
        float gamma;
    };

    struct PointLight
    {
        Vec3f position;
        Vec3f intensity;
    };

    struct AreaLight
    {
        Vec3f position;
        Vec3f normal;
        float extent;
        Vec3f radiance;
    };

    struct DirectionalLight
    {
        Vec3f direction;
        Vec3f radiance;
    };

    struct SpotLight
    {
        Vec3f position;
        Vec3f direction;
        Vec3f intensity;
        float coverageAngle;
        float falloffAngle;
    };

    struct SphericalDirectionalLight
    {
        int type; // 0: latlong - 1: probe
        int imageID;
    };

    struct Material
    {
        bool is_mirror;
        bool is_dielectric;
        bool is_conductor;
        bool has_phong;
        Vec3f ambient;
        Vec3f diffuse;
        Vec3f specular;
        Vec3f mirror;
        float phong_exponent;
        float refractionIndex;
        float absorbtionIndex;
        Vec3f absorbtionCoeff;
        bool hasRoughness;
        float roughness;
        int materialBRDFid;
    };

    struct BRDF
    {
        int type; // -1: None - 0: OriginalPhong - 1: ModifiedPhong - 2: OriginalBlinnPhong - 3: ModifiedBlinnPhong - 4: TorranceSparrow
        float exponent;
        bool isNormalized;
        bool isKdFresnel;
    };

    struct Texture
    {
        int width;
        int height;
        int channels;
        int type; // 0: image - 1: perlin

        int imageId;
        int decalMode; // 0: replaceBackground - 1: replace_kd - 2:replace_ks - 3: blend_kd - 4: replace_all - 5: replace_normal - 6: bump_normal
        int interpolation; // 0: bilinear - 1: nearest
        
        int noiseScale; // for perlin
        int noiseConversion; // for perlin-> 0: absval - 1: linear

        float bumpFactor;

        bool isHDRTexture;

        unsigned char* data;
        float* float_data;
    };

    struct ImageWithData
    {
        std::string imageName;
        int width;
        int height;
        int channels;
        unsigned char* data;
        float* float_data;
    };


    struct Face
    {
        int materialID;
        int v0_id;
        int v1_id;
        int v2_id;
        BoundingBox boundingBox;
        Matrix4x4 transformationMatrix;
        bool hasMotionBlur;
        Vec3f motionVector;
        std::vector<int> textureIDs;

    };

    struct Mesh
    {
        int material_id;
        std::vector<Face> faces;
        Matrix4x4 transformationMatrix;
        bool hasMotionBlur;
        Vec3f motionVector;
        std::vector<int> textureIDs;
    };

    struct Triangle
    {
        int material_id;
        Face indices;
        Matrix4x4 transformationMatrix;
        bool hasMotionBlur;
        Vec3f motionVector;
    };

    struct Sphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
        Matrix4x4 transformationMatrix;
        bool hasMotionBlur;
        Vec3f motionVector;
        std::vector<int> textureIDs;

    };

    struct LightSphere
    {
        int material_id;
        int center_vertex_id;
        float radius;
        Matrix4x4 transformationMatrix;
        Vec3f radiance;
    };

    struct LightMesh
    {
        int material_id;
        std::vector<Face> faces;
        Matrix4x4 transformationMatrix;
        Vec3f radiance;
    };

    struct Scene
    {
        //Data
        Vec3i background_color;
        float shadow_ray_epsilon;
        float intersection_test_epsilon;
        int max_recursion_depth;
        std::vector<Camera> cameras;
        Vec3f ambient_light;
        std::vector<PointLight> point_lights;
        std::vector<AreaLight> area_lights;
        std::vector<DirectionalLight> directional_lights;
        std::vector<SpotLight> spot_lights;
        std::vector<SphericalDirectionalLight> sphericalDirectional_lights;
        std::vector<Material> materials;
        std::vector<Vec3f> vertex_data;
        std::vector<Mesh> meshes;
        std::vector<Triangle> triangles;
        std::vector<Sphere> spheres;
        std::vector<Face> allMeshFaces; // for bvh
        std::vector<Matrix4x4> translations;
        std::vector<Matrix4x4> scalings;
        std::vector<Matrix4x4> rotations;
        std::vector<Vec2f> texCoord_data;
        std::vector<std::string> images;
        std::vector<ImageWithData> imageDataVector;
        std::vector<Texture> textures;
        std::vector<BRDF> brdfs;
        std::vector<LightSphere> lightSpheres;
        std::vector<LightMesh> lightMeshes;
        bool hasBackgroundTexture;
        bool hasAmbientLight;
        Texture backGroundTexture;
        ToneMapping sceneToneMap;

        //Functions
        void loadFromXml(const std::string &filepath);
        void readPlyFile(const std::string& filepath, const std::string& plyFile, Mesh& mesh, std::vector<parser::Vec3f> vertexData);
    };

}

#endif
