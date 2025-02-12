#include "parser.h"
#include "tinyxml2.h"
#include <sstream>
#include <stdexcept>
#include <cassert>
#include <iostream>
#include "happly.h"

void parser::Scene::readPlyFile(const std::string& filepath, const std::string& plyFile, Mesh& mesh, std::vector<parser::Vec3f> vertexData) {
    int pos = filepath.find_last_of("/");
	std::string plyDir(filepath.substr(0, pos + 1));
	std::string plyPath = plyDir + plyFile;


	happly::PLYData plyData(plyPath);

	std::vector<std::array<double, 3>> vertexPositions = plyData.getVertexPositions();
	std::vector<std::vector<int>> faceIndices = plyData.getFaceIndices<int>();

    

	int numberOfVertices = vertex_data.size();

    for (const auto& pos : vertexPositions) {
        Vec3f vertex;
        vertex.x = pos[0];
        vertex.y = pos[1];
        vertex.z = pos[2];

        vertex_data.push_back(vertex); 
    }

    for (const auto& face : faceIndices) {

        if(face.size() == 3) {
            Face triangle;
            triangle.v0_id = numberOfVertices + face[0] + 1;
            triangle.v1_id = numberOfVertices + face[1] + 1;
            triangle.v2_id = numberOfVertices + face[2] + 1;
            triangle.materialID = mesh.material_id;
            triangle.transformationMatrix = mesh.transformationMatrix; // transformation
            triangle.hasMotionBlur = mesh.hasMotionBlur;
            triangle.motionVector = mesh.motionVector;

            allMeshFaces.push_back(triangle);
            mesh.faces.push_back(triangle);
        }
        else if(face.size() == 4) {
            Face triangleOne;
            Face triangleTwo;

            triangleOne.v0_id = numberOfVertices + face[0] + 1;
            triangleOne.v1_id = numberOfVertices + face[1] + 1;
            triangleOne.v2_id = numberOfVertices + face[2] + 1;
            triangleOne.materialID = mesh.material_id;
            triangleOne.transformationMatrix = mesh.transformationMatrix; // transformation
            triangleOne.hasMotionBlur = mesh.hasMotionBlur;
            triangleOne.motionVector = mesh.motionVector;

            triangleTwo.v0_id = numberOfVertices + face[0] + 1;
            triangleTwo.v1_id = numberOfVertices + face[2] + 1;
            triangleTwo.v2_id = numberOfVertices + face[3] + 1;
            triangleTwo.materialID = mesh.material_id;
            triangleTwo.transformationMatrix = mesh.transformationMatrix; // transformation
            triangleTwo.hasMotionBlur = mesh.hasMotionBlur;
            triangleTwo.motionVector = mesh.motionVector;

            allMeshFaces.push_back(triangleOne);
            allMeshFaces.push_back(triangleTwo);
            mesh.faces.push_back(triangleOne);
            mesh.faces.push_back(triangleTwo);
        }
        
    }

    meshes.push_back(mesh);
    mesh.faces.clear();
}


void parser::Scene::loadFromXml(const std::string &filepath)
{
    tinyxml2::XMLDocument file;
    std::stringstream stream;

    auto res = file.LoadFile(filepath.c_str());
    if (res)
    {
        throw std::runtime_error("Error: The xml file cannot be loaded.");
    }

    auto root = file.FirstChild();
    if (!root)
    {
        throw std::runtime_error("Error: Root is not found.");
    }

    //Get BackgroundColor
    auto element = root->FirstChildElement("BackgroundColor");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0 0 0" << std::endl;
    }
    stream >> background_color.x >> background_color.y >> background_color.z;


    //Get ShadowRayEpsilon
    element = root->FirstChildElement("ShadowRayEpsilon");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0.001" << std::endl;
    }
    stream >> shadow_ray_epsilon;

    //Get IntersectionTestEpsilon
    element = root->FirstChildElement("IntersectionTestEpsilon");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0.001" << std::endl;
    }
    stream >> intersection_test_epsilon;

    //Get MaxRecursionDepth
    element = root->FirstChildElement("MaxRecursionDepth");
    if (element)
    {
        stream << element->GetText() << std::endl;
    }
    else
    {
        stream << "0" << std::endl;
    }
    stream >> max_recursion_depth;   

    

    //Get Cameras
    element = root->FirstChildElement("Cameras");
    element = element->FirstChildElement("Camera");
    Camera camera;
    while (element)
    {
        auto type = element->Attribute("type");
        if(!type) {
            auto child = element->FirstChildElement("Position");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Gaze");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Up");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("NearPlane");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("NearDistance");
            stream << child->GetText() << std::endl;

            // dof effect
            camera.hasDOF = false;
            child = element->FirstChildElement("FocusDistance");
            if(child) {
                stream << child->GetText() << std::endl; // focus distance
                child = element->FirstChildElement("ApertureSize");
                stream << child->GetText() << std::endl; // aperture size
                camera.hasDOF = true;
            }

            child = element->FirstChildElement("ImageResolution");
            stream << child->GetText() << std::endl;

            bool hasSamples = false;
            child = element->FirstChildElement("NumSamples");
            if(child) {
                stream << child->GetText() << std::endl;
                hasSamples = true;
            }

            child = element->FirstChildElement("ImageName");
            stream << child->GetText() << std::endl;

            stream >> camera.position.x >> camera.position.y >> camera.position.z;
            stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
            stream >> camera.up.x >> camera.up.y >> camera.up.z;
            stream >> camera.near_plane.x >> camera.near_plane.y >> camera.near_plane.z >> camera.near_plane.w;
            stream >> camera.near_distance;

            // dof effect
            if(camera.hasDOF) {
                stream >> camera.focusDistance;
                stream >> camera.apertureSize;
            }

            stream >> camera.image_width >> camera.image_height;

            if(hasSamples) {
                stream >> camera.numSamples;
            }
            else {
                camera.numSamples = 1;
            }

            stream >> camera.image_name;

            // tonemapping
            auto toneMapChild = element->FirstChildElement("Tonemap");
            ToneMapping toneMap;
            toneMap.hasToneMapping = false;
            if(toneMapChild) {
                toneMap.hasToneMapping = true;
                child = toneMapChild->FirstChildElement("TMOOptions");
                stream << child->GetText() << std::endl;

                child = toneMapChild->FirstChildElement("Saturation");
                stream << child->GetText() << std::endl;

                child = toneMapChild->FirstChildElement("Gamma");
                stream << child->GetText() << std::endl;

                stream >> toneMap.keyValue;
                stream >> toneMap.burnPercent;
                stream >> toneMap.saturation;
                stream >> toneMap.gamma;
            }
            sceneToneMap = toneMap;
            

            cameras.push_back(camera);
            element = element->NextSiblingElement("Camera");
        }
        else if(strcmp(type,"lookAt") == 0) {
            auto child = element->FirstChildElement("Position");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("GazePoint");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Up");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("FovY");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("NearDistance");
            stream << child->GetText() << std::endl;

            // dof effect
            camera.hasDOF = false;
            child = element->FirstChildElement("FocusDistance");
            if(child) {
                stream << child->GetText() << std::endl; // focus distance
                child = element->FirstChildElement("ApertureSize");
                stream << child->GetText() << std::endl; // aperture size
                camera.hasDOF = true;
            }

            child = element->FirstChildElement("ImageResolution");
            stream << child->GetText() << std::endl;

            bool hasSamples = false;
            child = element->FirstChildElement("NumSamples");
            if(child) {
                stream << child->GetText() << std::endl;
                hasSamples = true;
            }
        
            child = element->FirstChildElement("ImageName");
            stream << child->GetText() << std::endl;


            stream >> camera.position.x >> camera.position.y >> camera.position.z;
            stream >> camera.gaze.x >> camera.gaze.y >> camera.gaze.z;
            stream >> camera.up.x >> camera.up.y >> camera.up.z;
            stream >> camera.fovy;
            stream >> camera.near_distance;

            // dof effect
            if(camera.hasDOF) {
                stream >> camera.focusDistance;
                stream >> camera.apertureSize;
            }
            

            stream >> camera.image_width >> camera.image_height;

            if(hasSamples) {
                stream >> camera.numSamples;
            }
            else {
                camera.numSamples = 1;
            }

            stream >> camera.image_name;

            
            // gaze point to gaze vector
            camera.gaze.x = camera.gaze.x - camera.position.x;
            camera.gaze.y = camera.gaze.y - camera.position.y;
            camera.gaze.z = camera.gaze.z - camera.position.z;
            float distance = sqrtf(powf(camera.gaze.x,2) + powf(camera.gaze.y,2) + powf(camera.gaze.z,2));
            camera.gaze.x = camera.gaze.x / distance;
            camera.gaze.y = camera.gaze.y / distance;
            camera.gaze.z = camera.gaze.z / distance;

            camera.fovy = camera.fovy * (3.14f / 180.0f); // Convert FOV from degrees to radians
            float aspectRatio = static_cast<float>(camera.image_width) / static_cast<float>(camera.image_height);
            // Calculate the boundaries of the image plane 
            // left rigth bottom top
            camera.near_plane.w = camera.near_distance * std::tan(camera.fovy / 2.0f);
            camera.near_plane.z = -camera.near_plane.w;
            camera.near_plane.y = camera.near_plane.w * aspectRatio;
            camera.near_plane.x = -camera.near_plane.y;

            // tonemapping
            auto toneMapChild = element->FirstChildElement("Tonemap");
            ToneMapping toneMap;
            toneMap.hasToneMapping = false;
            if(toneMapChild) {
                toneMap.hasToneMapping = true;
                child = toneMapChild->FirstChildElement("TMOOptions");
                stream << child->GetText() << std::endl;

                child = toneMapChild->FirstChildElement("Saturation");
                stream << child->GetText() << std::endl;

                child = toneMapChild->FirstChildElement("Gamma");
                stream << child->GetText() << std::endl;

                stream >> toneMap.keyValue;
                stream >> toneMap.burnPercent;
                stream >> toneMap.saturation;
                stream >> toneMap.gamma;
            }
            sceneToneMap = toneMap;

            cameras.push_back(camera);
            element = element->NextSiblingElement("Camera");
        }
        
    }
    stream.clear();


    //Get Lights
    element = root->FirstChildElement("Lights");
    if(element) {
        auto child = element->FirstChildElement("AmbientLight");
        hasAmbientLight = false;
        if(child) {
            stream << child->GetText() << std::endl;
            stream >> ambient_light.x >> ambient_light.y >> ambient_light.z;
            hasAmbientLight = true;
        }
        element = element->FirstChildElement("PointLight");
        PointLight point_light;
        while (element)
        {
            child = element->FirstChildElement("Position");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Intensity");
            stream << child->GetText() << std::endl;

            stream >> point_light.position.x >> point_light.position.y >> point_light.position.z;
            stream >> point_light.intensity.x >> point_light.intensity.y >> point_light.intensity.z;

            point_lights.push_back(point_light);
            element = element->NextSiblingElement("PointLight");
        }
        stream.clear();

        // area lights
        element = root->FirstChildElement("Lights");
        element = element->FirstChildElement("AreaLight");
        AreaLight area_light;
        while (element)
        {
            child = element->FirstChildElement("Position");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Normal");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Size");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Radiance");
            stream << child->GetText() << std::endl;

            
            stream >> area_light.position.x >> area_light.position.y >> area_light.position.z;
            stream >> area_light.normal.x >> area_light.normal.y >> area_light.normal.z;
            stream >> area_light.extent;
            stream >> area_light.radiance.x >> area_light.radiance.y >> area_light.radiance.z;

            area_lights.push_back(area_light);
            element = element->NextSiblingElement("AreaLight");
        }
        stream.clear();

        // directional lights
        element = root->FirstChildElement("Lights");
        element = element->FirstChildElement("DirectionalLight");
        DirectionalLight directional_light;
        while (element)
        {
            child = element->FirstChildElement("Direction");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Radiance");
            stream << child->GetText() << std::endl;
            
            stream >> directional_light.direction.x >> directional_light.direction.y >> directional_light.direction.z;
            stream >> directional_light.radiance.x >> directional_light.radiance.y >> directional_light.radiance.z;

            directional_lights.push_back(directional_light);
            element = element->NextSiblingElement("DirectionalLight");
        }
        stream.clear();

        // spot lights
        element = root->FirstChildElement("Lights");
        element = element->FirstChildElement("SpotLight");
        SpotLight spot_light;
        while (element)
        {
            child = element->FirstChildElement("Position");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Direction");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("Intensity");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("CoverageAngle");
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("FalloffAngle");
            stream << child->GetText() << std::endl;

            
            stream >> spot_light.position.x >> spot_light.position.y >> spot_light.position.z;
            stream >> spot_light.direction.x >> spot_light.direction.y >> spot_light.direction.z;
            stream >> spot_light.intensity.x >> spot_light.intensity.y >> spot_light.intensity.z;
            stream >> spot_light.coverageAngle;
            stream >> spot_light.falloffAngle;

            spot_lights.push_back(spot_light);
            element = element->NextSiblingElement("SpotLight");
        }
        stream.clear();

        // spherical directional lights
        element = root->FirstChildElement("Lights");
        element = element->FirstChildElement("SphericalDirectionalLight");
        SphericalDirectionalLight sphericalDirectionalLight;
        while (element)
        {
            auto lightType = element->Attribute("type");
            if(lightType != nullptr) {
                if(strcmp(lightType,"latlong") == 0) {
                    sphericalDirectionalLight.type = 0;
                }
                else if(strcmp(lightType,"probe") == 0) {
                    sphericalDirectionalLight.type = 1;
                }
                else {
                    // by default, latlong
                    sphericalDirectionalLight.type = 0;
                }
            }
            else {
                // by default, latlong
                sphericalDirectionalLight.type = 0;
            }
            
            child = element->FirstChildElement("ImageId");
            stream << child->GetText() << std::endl;
        
            stream >> sphericalDirectionalLight.imageID;

            sphericalDirectional_lights.push_back(sphericalDirectionalLight);
            element = element->NextSiblingElement("SphericalDirectionalLight");
        }
    }
    stream.clear();

    

    // Get BRDFs
    auto elementBRDF = root->FirstChildElement("BRDFs");
    if (elementBRDF)
    {

        auto originalPhong = elementBRDF->FirstChildElement("OriginalPhong");
        while (originalPhong)
        {
            BRDF currentBRDF;
            currentBRDF.isNormalized = false;
            currentBRDF.type = 0;

            auto exponentElem = originalPhong->FirstChildElement("Exponent");
            if (exponentElem && exponentElem->GetText())
            {
                std::string exponentStr = exponentElem->GetText(); 

                std::stringstream sstream(exponentStr);
                sstream >> currentBRDF.exponent;


                brdfs.push_back(currentBRDF);
            }

            originalPhong = originalPhong->NextSiblingElement("OriginalPhong");
        }

        auto modifiedPhong = elementBRDF->FirstChildElement("ModifiedPhong");
        while (modifiedPhong)
        {
            BRDF currentBRDF;
            currentBRDF.isNormalized = false;
            currentBRDF.type = 1;

            currentBRDF.isNormalized = (modifiedPhong->Attribute("normalized", "true") != NULL);

            auto exponentElem = modifiedPhong->FirstChildElement("Exponent");
            if (exponentElem && exponentElem->GetText())
            {
                std::string exponentStr = exponentElem->GetText(); 

                std::stringstream sstream(exponentStr);
                sstream >> currentBRDF.exponent;


                brdfs.push_back(currentBRDF);
            }

            modifiedPhong = modifiedPhong->NextSiblingElement("ModifiedPhong");
        }

        auto originalBlinnPhong = elementBRDF->FirstChildElement("OriginalBlinnPhong");
        while (originalBlinnPhong)
        {
            BRDF currentBRDF;
            currentBRDF.isNormalized = false;
            currentBRDF.type = 2;

            auto exponentElem = originalBlinnPhong->FirstChildElement("Exponent");
            if (exponentElem && exponentElem->GetText())
            {
                std::string exponentStr = exponentElem->GetText(); 

                std::stringstream sstream(exponentStr);
                sstream >> currentBRDF.exponent;


                brdfs.push_back(currentBRDF);
            }

            originalBlinnPhong = originalBlinnPhong->NextSiblingElement("OriginalBlinnPhong");
        }

        auto modifiedBlinnPhong = elementBRDF->FirstChildElement("ModifiedBlinnPhong");
        while (modifiedBlinnPhong)
        {
            BRDF currentBRDF;
            currentBRDF.isNormalized = false;
            currentBRDF.type = 3;

            currentBRDF.isNormalized = (modifiedBlinnPhong->Attribute("normalized", "true") != NULL);

            auto exponentElem = modifiedBlinnPhong->FirstChildElement("Exponent");
            if (exponentElem && exponentElem->GetText())
            {
                std::string exponentStr = exponentElem->GetText(); 

                std::stringstream sstream(exponentStr);
                sstream >> currentBRDF.exponent;


                brdfs.push_back(currentBRDF);
            }

            modifiedBlinnPhong = modifiedBlinnPhong->NextSiblingElement("ModifiedBlinnPhong");
        }

        auto torrenceSparrow = elementBRDF->FirstChildElement("TorranceSparrow");
        while (torrenceSparrow)
        {
            BRDF currentBRDF;
            currentBRDF.isNormalized = false;
            currentBRDF.type = 4;
            currentBRDF.isKdFresnel = false;

            currentBRDF.isKdFresnel = (torrenceSparrow->Attribute("kdfresnel", "true") != NULL);

            auto exponentElem = torrenceSparrow->FirstChildElement("Exponent");
            if (exponentElem && exponentElem->GetText())
            {
                std::string exponentStr = exponentElem->GetText(); 

                std::stringstream sstream(exponentStr);
                sstream >> currentBRDF.exponent;


                brdfs.push_back(currentBRDF);
            }

            torrenceSparrow = torrenceSparrow->NextSiblingElement("TorranceSparrow");
        }

    }
    stream.clear();


    //Get Materials
    element = root->FirstChildElement("Materials");
    element = element->FirstChildElement("Material");
    Material material;
    while (element)
    {
        bool isDegamma = false;
        isDegamma = (element->Attribute("degamma", "true") != NULL);

        material.is_mirror = (element->Attribute("type", "mirror") != NULL);
        material.is_conductor = (element->Attribute("type", "conductor") != NULL);
        material.is_dielectric = (element->Attribute("type", "dielectric") != NULL);

        material.materialBRDFid = -1;
        stream << element->Attribute("BRDF");
        stream >> material.materialBRDFid;
        stream.clear();


        auto child = element->FirstChildElement("AmbientReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("DiffuseReflectance");
        stream << child->GetText() << std::endl;
        child = element->FirstChildElement("SpecularReflectance");
        stream << child->GetText() << std::endl;
        

        

        if(material.is_mirror || material.is_conductor) {
            child = element->FirstChildElement("MirrorReflectance");
            stream << child->GetText() << std::endl;
        }
            

        

        material.has_phong = (element->FirstChildElement("PhongExponent") != NULL);
        if(material.has_phong) {
            child = element->FirstChildElement("PhongExponent");
            stream << child->GetText() << std::endl;
        }

        // torrance-sparrow
        child = element->FirstChildElement("RefractionIndex");
        bool isTorrance = false;
        if(child) {
            stream << child->GetText() << std::endl;
            child = element->FirstChildElement("AbsorptionIndex");
            stream << child->GetText() << std::endl;
            isTorrance = true;
        }

        // refraction and absorbtion
        if(material.is_conductor || material.is_dielectric) {
            child = element->FirstChildElement("RefractionIndex");
            if (child)
            {
                stream << child->GetText() << std::endl;
            }
        }
        

        if(material.is_conductor) {
            child = element->FirstChildElement("AbsorptionIndex");
            if (child)
            {
                stream << child->GetText() << std::endl;
            }
        }
        

        if(material.is_dielectric) {
            child = element->FirstChildElement("AbsorptionCoefficient"); 
            if (child)
            {
                stream << child->GetText() << std::endl;
            }
        }

        child = element->FirstChildElement("Roughness"); 
        if (child)
        {
            stream << child->GetText() << std::endl;
            material.hasRoughness = true;
        }
    
        
        stream >> material.ambient.x >> material.ambient.y >> material.ambient.z;
        stream >> material.diffuse.x >> material.diffuse.y >> material.diffuse.z;
        stream >> material.specular.x >> material.specular.y >> material.specular.z;

        if(isDegamma) {
            material.ambient.x = pow(material.ambient.x,sceneToneMap.gamma);
            material.ambient.y = pow(material.ambient.y,sceneToneMap.gamma);
            material.ambient.z = pow(material.ambient.z,sceneToneMap.gamma);

            material.diffuse.x = pow(material.diffuse.x,sceneToneMap.gamma);
            material.diffuse.y = pow(material.diffuse.y,sceneToneMap.gamma);
            material.diffuse.z = pow(material.diffuse.z,sceneToneMap.gamma);

            material.specular.x = pow(material.specular.x,sceneToneMap.gamma);
            material.specular.y = pow(material.specular.y,sceneToneMap.gamma);
            material.specular.z = pow(material.specular.z,sceneToneMap.gamma);
        }

        
        if (material.is_mirror || material.is_conductor)
        {
            stream >> material.mirror.x >> material.mirror.y >> material.mirror.z;
        }

        if(material.has_phong) {
            stream >> material.phong_exponent;
        }

        if(isTorrance) {
            stream >> material.refractionIndex;
            stream >> material.absorbtionIndex;
        }

        if(material.is_conductor) {
            stream >> material.refractionIndex;
            stream >> material.absorbtionIndex;
        }

        if(material.is_dielectric) {
            stream >> material.refractionIndex;
            stream >> material.absorbtionCoeff.x >> material.absorbtionCoeff.y >> material.absorbtionCoeff.z;
        }

        if(material.hasRoughness) {
            stream >> material.roughness;
        }
        
        
        materials.push_back(material);
        element = element->NextSiblingElement("Material");
        stream.clear();
    }
    stream.clear();


    //Get Textures
    hasBackgroundTexture = false;
    element = root->FirstChildElement("Textures");
    if(element) {
        auto child = element->FirstChildElement("Images");
        if(child) {
            child = child->FirstChildElement("Image");
			while (child) {
				std::string image;
                ImageWithData imageData;
				stream << child->GetText() << std::endl;
				stream >> image;
                imageData.imageName = image;
				images.push_back(image);
                imageDataVector.push_back(imageData);
				child = child->NextSiblingElement("Image");
			}
        }

        Texture currentTexture;
        currentTexture.isHDRTexture = false;
        currentTexture.imageId = -1;
        currentTexture.interpolation = -1;
        currentTexture.noiseConversion = -1;
        currentTexture.noiseScale = -1;
        currentTexture.decalMode = -1;
        currentTexture.bumpFactor = 1;
        element = element->FirstChildElement("TextureMap");
		while (element) {

            std::string textureType = element->Attribute("type");
            if(strcmp(textureType.c_str(),"image") == 0) {
                currentTexture.type = 0;
            }
            else if(strcmp(textureType.c_str(),"perlin") == 0) {
                currentTexture.type = 1;
            }
            else {
                currentTexture.type = -1;
            }

            if(currentTexture.type == 0) {
                // image textures
                child = element->FirstChildElement("ImageId");
                stream << child->GetText() << std::endl;
                stream >> currentTexture.imageId;

                std::string interpolationType;
                child = element->FirstChildElement("Interpolation");
                if(child) {
                    stream << child->GetText() << std::endl;
                    stream >> interpolationType;
                    if(strcmp(interpolationType.c_str(),"bilinear") == 0) {
                        currentTexture.interpolation = 0;
                    }
                    else if(strcmp(interpolationType.c_str(),"nearest") == 0) {
                        currentTexture.interpolation = 1;
                    }
                    else {
                        currentTexture.interpolation = -1;
                    }
                }
                
                std::string decalMode;
                child = element->FirstChildElement("DecalMode");
                stream << child->GetText() << std::endl;
                stream >> decalMode;
                if(strcmp(decalMode.c_str(),"replace_background") == 0) {
                    currentTexture.decalMode = 0;
                    hasBackgroundTexture = true;
                    backGroundTexture = currentTexture;
                }
                else if(strcmp(decalMode.c_str(),"replace_kd") == 0) {
                    currentTexture.decalMode = 1;
                    
                }
                else if(strcmp(decalMode.c_str(),"replace_ks") == 0) {
                    currentTexture.decalMode = 2;
                    
                }
                else if(strcmp(decalMode.c_str(),"blend_kd") == 0) {
                    currentTexture.decalMode = 3;
                }
                else if(strcmp(decalMode.c_str(),"replace_all") == 0) {
                    currentTexture.decalMode = 4;
                }
                else if(strcmp(decalMode.c_str(),"replace_normal") == 0) {
                    currentTexture.decalMode = 5;
                    currentTexture.interpolation = 0;
                }
                else if(strcmp(decalMode.c_str(),"bump_normal") == 0) {
                    currentTexture.decalMode = 6;
                    currentTexture.interpolation = 0;
                    auto factor = element->FirstChildElement("BumpFactor");
                    if(factor) {
                        stream << factor->GetText() << std::endl;
                        stream >> currentTexture.bumpFactor;
                    }
                }
                else {
                    currentTexture.decalMode = -1;
                }
            }
            else if(currentTexture.type == 1) {
                // perlin textures
                
                child = element->FirstChildElement("NoiseScale");
                if(child) {
                    stream << child->GetText() << std::endl;
                    stream >> currentTexture.noiseScale;
                }
                

                std::string noiseConversion;
                child = element->FirstChildElement("NoiseConversion");
                stream << child->GetText() << std::endl;
                stream >> noiseConversion;
                if(strcmp(noiseConversion.c_str(),"absval") == 0) {
                    currentTexture.noiseConversion = 0;
                }
                else if(strcmp(noiseConversion.c_str(),"linear") == 0) {
                    currentTexture.noiseConversion = 1;
                }
                else {
                    currentTexture.noiseConversion = -1;
                }

                std::string decalMode;
                child = element->FirstChildElement("DecalMode");
                stream << child->GetText() << std::endl;
                stream >> decalMode;
                if(strcmp(decalMode.c_str(),"replace_background") == 0) {
                    currentTexture.decalMode = 0;
                    hasBackgroundTexture = true;
                    backGroundTexture = currentTexture;
                }
                else if(strcmp(decalMode.c_str(),"replace_kd") == 0) {
                    currentTexture.decalMode = 1;
                    
                }
                else if(strcmp(decalMode.c_str(),"replace_ks") == 0) {
                    currentTexture.decalMode = 2;
                    
                }
                else if(strcmp(decalMode.c_str(),"blend_kd") == 0) {
                    currentTexture.decalMode = 3;
                }
                else if(strcmp(decalMode.c_str(),"replace_all") == 0) {
                    currentTexture.decalMode = 4;
                }
                else {
                    currentTexture.decalMode = -1;
                }
            }

            textures.push_back(currentTexture);
            element = element->NextSiblingElement("TextureMap");
        }

    }
    stream.clear();

    //Get Scalings
    element = root->FirstChildElement("Transformations");
    if(element) {
        auto child = element->FirstChildElement("Scaling");
        Matrix4x4 scalingMatrix;
        while (child)
        {
            float sx,sy,sz;
            stream << child->GetText() << std::endl;
            stream >> sx >> sy >> sz;

            scalingMatrix = scalingMatrix.ScalingMatrix(sx,sy,sz);
            scalings.push_back(scalingMatrix);
            child = child->NextSiblingElement("Scaling");
        }
    }
    stream.clear();

    //Get Translation
    element = root->FirstChildElement("Transformations");
    if(element) {
        auto child = element->FirstChildElement("Translation");
        Matrix4x4 translationMatrix;
        while (child)
        {
            float tx,ty,tz;
            stream << child->GetText() << std::endl;
            stream >> tx >> ty >> tz;

            translationMatrix = translationMatrix.TranslationMatrix(tx,ty,tz);
            translations.push_back(translationMatrix);
            child = child->NextSiblingElement("Translation");
        }
    }
    stream.clear();

    //Get Rotation
    element = root->FirstChildElement("Transformations");
    if(element) {
        auto child = element->FirstChildElement("Rotation");
        Matrix4x4 rotationMatrix;
        while (child)
        {
            float theta,rx,ry,rz;
            stream << child->GetText() << std::endl;
            stream >> theta >> rx >> ry >> rz;

            rotationMatrix = rotationMatrix.RotationMatrix(theta,rx,ry,rz);
            rotations.push_back(rotationMatrix);
            child = child->NextSiblingElement("Rotation");
        }
    }
    stream.clear();


    //Get VertexData
    element = root->FirstChildElement("VertexData");
    if(element) {
        stream << element->GetText() << std::endl;
        Vec3f vertex;
        while (!(stream >> vertex.x).eof())
        {
            stream >> vertex.y >> vertex.z;
            vertex_data.push_back(vertex);
        }
    }
    stream.clear();

    //Get TexCoordData
    element = root->FirstChildElement("TexCoordData");
    if(element) {
        stream << element->GetText() << std::endl;
        Vec2f texCoord;
        while (!(stream >> texCoord.u).eof())
        {
            stream >> texCoord.v;
            texCoord_data.push_back(texCoord);
        }
    }
    stream.clear();

    //Get Meshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Mesh");
    Mesh mesh;
    while (element)
    {
        auto child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> mesh.material_id;

        //textures
        child = element->FirstChildElement("Textures");
        if(child) {
            std::string textureText = child->GetText();
            std::stringstream stream(textureText);

            std::vector<int> textureIDs;
            int textureID;

            while (stream >> textureID) {
                textureIDs.push_back(textureID);
            }

            mesh.textureIDs = textureIDs;
        }
        else {
            mesh.textureIDs.clear();
        }
        stream.clear();
        

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;
        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        mesh.transformationMatrix = compositeMatrix;
        stream.clear();

        child = element->FirstChildElement("MotionBlur");
        if(child) {

            stream << child->GetText() << std::endl;
            stream >> mesh.motionVector.x >> mesh.motionVector.y >> mesh.motionVector.z;

            mesh.hasMotionBlur = true;
        }
        else {
            mesh.hasMotionBlur = false;
        }

        child = element->FirstChildElement("Faces");
        auto plyFile = child->Attribute("plyFile");
        int vertexoffset = 0;
        stream << child->Attribute("vertexOffset");
        stream >> vertexoffset;

        int textureoffset = 0;
        stream << child->Attribute("textureOffset");
        stream >> textureoffset;

        stream.clear();
        //std::cout << vertexoffset;
		if (!plyFile) {
            stream << child->GetText() << std::endl;
            Face face;
            face.transformationMatrix = mesh.transformationMatrix; // for transformation
            while (!(stream >> face.v0_id).eof())
            {
                stream >> face.v1_id >> face.v2_id;
                face.materialID = mesh.material_id; // for BVH
                face.v0_id += vertexoffset; 
                face.v1_id += vertexoffset; 
                face.v2_id += vertexoffset; 
                face.hasMotionBlur = mesh.hasMotionBlur;
                face.motionVector = mesh.motionVector;

                for(auto textureid: mesh.textureIDs) {
                    face.textureIDs.push_back(textureid + textureoffset);
                }

                allMeshFaces.push_back(face); // for BVH
                mesh.faces.push_back(face);
            }
            stream.clear();

            meshes.push_back(mesh);
            mesh.faces.clear();
            element = element->NextSiblingElement("Mesh");
        }
        else {

            // add object pushback
            readPlyFile(filepath,plyFile,mesh,vertex_data);
            stream.clear();
            mesh.faces.clear();
            element = element->NextSiblingElement("Mesh");
        }
        
    }
    stream.clear();

    //Get LightMeshes
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("LightMesh");
    LightMesh lightMesh;
    while (element)
    {
        auto child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> lightMesh.material_id;
        

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;
        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        lightMesh.transformationMatrix = compositeMatrix;
        stream.clear();

        child = element->FirstChildElement("Radiance");
        stream << child->GetText() << std::endl;
        stream >> lightMesh.radiance.x >> lightMesh.radiance.y >> lightMesh.radiance.z;

        child = element->FirstChildElement("Faces");
        int vertexoffset = 0;
        stream << child->Attribute("vertexOffset");
        stream >> vertexoffset;
        stream.clear();
		
        stream << child->GetText() << std::endl;
        Face face;
        face.transformationMatrix = lightMesh.transformationMatrix; // for transformation
        while (!(stream >> face.v0_id).eof())
        {
            stream >> face.v1_id >> face.v2_id;
            face.materialID = lightMesh.material_id; // for BVH
            face.v0_id += vertexoffset; 
            face.v1_id += vertexoffset; 
            face.v2_id += vertexoffset; 

            lightMesh.faces.push_back(face);
        }
        stream.clear();

        lightMeshes.push_back(lightMesh);
        lightMesh.faces.clear();
        element = element->NextSiblingElement("LightMesh");
    }
    stream.clear();

    //Get MeshInstances
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("MeshInstance");
    Mesh meshInstance;
    while (element)
    {
        bool hasSpecialMaterial = false;
        if(element->FirstChildElement("Material")) {
            auto child = element->FirstChildElement("Material");
            stream << child->GetText() << std::endl;
            stream >> meshInstance.material_id;
            hasSpecialMaterial = true;
        }
        
        int id = atoi(element->Attribute("id")) - 1;

        int baseMeshID = atoi(element->Attribute("baseMeshId"));
        Mesh baseMesh = meshes[baseMeshID - 1];

        if(hasSpecialMaterial == false) {
            meshInstance.material_id = baseMesh.material_id;
        }

        const char* isResetTransform = "false";
        if(element->Attribute("resetTransform")) {
            isResetTransform = element->Attribute("resetTransform");
        }

        auto child = element->FirstChildElement("MotionBlur");
        if(child) {

            stream << child->GetText() << std::endl;
            stream >> meshInstance.motionVector.x >> meshInstance.motionVector.y >> meshInstance.motionVector.z;

            meshInstance.hasMotionBlur = true;
        }
        else {
            meshInstance.hasMotionBlur = false;
        }

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;

        if(strcmp(isResetTransform,"true") != 0) {
            compositeMatrix = baseMesh.transformationMatrix;
        }

        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        meshInstance.transformationMatrix = compositeMatrix;
        stream.clear();

        meshInstance.faces = baseMesh.faces;

        // update face transformations, materialid
        for(int i = 0; i < meshInstance.faces.size(); i++) {
            Face currentFace = meshInstance.faces[i];
            currentFace.materialID = meshInstance.material_id;
            currentFace.transformationMatrix = meshInstance.transformationMatrix;
            currentFace.motionVector = meshInstance.motionVector;
            currentFace.hasMotionBlur = meshInstance.hasMotionBlur;
            
            meshInstance.faces[i] = currentFace;
        }

        meshes.insert(meshes.begin() + id, meshInstance);
        meshInstance.faces.clear();
        stream.clear();
        element = element->NextSiblingElement("MeshInstance");
    }
    stream.clear();

    //Get Triangles
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Triangle");
    Triangle triangle;
    while (element)
    {
        auto child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> triangle.material_id;

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;
        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        triangle.transformationMatrix = compositeMatrix;
        triangle.indices.transformationMatrix = compositeMatrix;
        stream.clear();

        child = element->FirstChildElement("Indices");
        stream << child->GetText() << std::endl;
        stream >> triangle.indices.v0_id >> triangle.indices.v1_id >> triangle.indices.v2_id;

        triangles.push_back(triangle);
        element = element->NextSiblingElement("Triangle");
    }

    //Get Sphere Lights
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("LightSphere");
    LightSphere lightSphere;
    while (element)
    {
        auto child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> lightSphere.material_id;

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;
        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        lightSphere.transformationMatrix = compositeMatrix;
        stream.clear();

        child = element->FirstChildElement("Center");
        stream << child->GetText() << std::endl;
        stream >> lightSphere.center_vertex_id;

        child = element->FirstChildElement("Radius");
        stream << child->GetText() << std::endl;
        stream >> lightSphere.radius;

        child = element->FirstChildElement("Radius");
        stream << child->GetText() << std::endl;
        stream >> lightSphere.radius;

        child = element->FirstChildElement("Radiance");
        stream << child->GetText() << std::endl;
        stream >> lightSphere.radiance.x >> lightSphere.radiance.y >> lightSphere.radiance.z;

        lightSpheres.push_back(lightSphere);
        element = element->NextSiblingElement("LightSphere");
    }
    stream.clear();

    //Get Spheres
    element = root->FirstChildElement("Objects");
    element = element->FirstChildElement("Sphere");
    Sphere sphere;
    while (element)
    {
        auto child = element->FirstChildElement("Material");
        stream << child->GetText() << std::endl;
        stream >> sphere.material_id;

        //textures
        child = element->FirstChildElement("Textures");
        if(child) {
            std::string textureText = child->GetText();
            std::stringstream stream(textureText);

            std::vector<int> textureIDs;
            int textureID;

            while (stream >> textureID) {
                textureIDs.push_back(textureID);
            }

            sphere.textureIDs = textureIDs;
        }
        else {
            sphere.textureIDs.clear();
        }
        stream.clear();

        //transformations
        std::string transforms;
        Matrix4x4 compositeMatrix = Matrix4x4();
        compositeMatrix.elements[0][0] = 1;
        compositeMatrix.elements[1][1] = 1;
        compositeMatrix.elements[2][2] = 1;
        compositeMatrix.elements[3][3] = 1;
        child = element->FirstChildElement("Transformations");
        if(child) {
            stream << child->GetText() << std::endl;
            while(!(stream >> transforms).eof()) {
                
                if(transforms[0] == 's') {
                    compositeMatrix = compositeMatrix.multiply(scalings[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 't') {
                    compositeMatrix = compositeMatrix.multiply(translations[((int) transforms[1] - '0') - 1]);
                }
                else if(transforms[0] == 'r'){
                    compositeMatrix = compositeMatrix.multiply(rotations[((int) transforms[1] - '0') - 1]);
                }
            }
        }
        sphere.transformationMatrix = compositeMatrix;
        stream.clear();

        child = element->FirstChildElement("Center");
        stream << child->GetText() << std::endl;
        stream >> sphere.center_vertex_id;

        child = element->FirstChildElement("Radius");
        stream << child->GetText() << std::endl;
        stream >> sphere.radius;

        spheres.push_back(sphere);
        element = element->NextSiblingElement("Sphere");
    }


}
