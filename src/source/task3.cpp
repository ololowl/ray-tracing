#include <iostream>
#include <string>
#include <vector>
#include <cfloat>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <limits>

#include "EasyBMP.h"
#include "tiny_obj_loader.h"
#include "glm/glm/glm.hpp"
#include "glm/glm/gtc/random.hpp"
#include "glm/glm/gtx/intersect.hpp"
#include "glm/glm/gtx/normal.hpp"
#include "glm/glm/gtx/norm.hpp"
#include "glm/glm/geometric.hpp"

const int NUM_THREADS = 16;
constexpr float EPS = std::numeric_limits<float>::epsilon();

struct MyTriangle {
    glm::vec3 p[3];
    MyTriangle() {
    	for (int i = 0; i < 3; ++i) {
			p[i] = glm::vec3(0.0f);
		}
    }
    MyTriangle(glm::vec3 *point) {
        p[0] = point[0];
        p[1] = point[1];
        p[2] = point[2];
    }
    MyTriangle &operator= (const MyTriangle &t) = default;
};

struct Params {
    glm::vec3 max, min; // coordinates
    glm::vec3 voxelSize;
    static const int NUM_VOXELS = 300;

    Params () {
    	max = glm::vec3(0.0f);
    	min = glm::vec3(FLT_MAX);
    	voxelSize = glm::vec3(0.0f);
    }
};

struct Intersection {
	MyTriangle triangle;
	float dist;
	glm::vec2 bary;
	glm::vec3 point;
	glm::vec3 ray;
	Intersection () = default;
	void get_point () {
		point = triangle.p[0] * (1 - bary[0] - bary[1]) + triangle.p[1] * bary[0] + 
			 	triangle.p[2] * bary[1];
	}
	glm::vec3 normal() {
		return glm::triangleNormal(triangle.p[0], triangle.p[1], triangle.p[2]);
	}
};

struct CoordPointers {
	float *x, *y, *z;
};

struct WiFiSource {
    glm::vec3 p;
    float power;
    float maxRadius;
};

struct Camera {
    glm::vec3 p;
    glm::vec3 viewDir;
    float viewAngleX, viewAngleY;
    glm::vec3 upDir, rightDir;
    float distCameraProjection;
    float imageHeight, imageWidth; // pixels
    float realHeight, realWidth;
    float pixHeight, pixWidth; // one pixel
    void set_evrthng () {
    	distCameraProjection = glm::l2Norm(viewDir);
    	upDir *= distCameraProjection * std::tan(viewAngleY / 2) / glm::l2Norm(upDir);
    	rightDir *= distCameraProjection * std::tan(viewAngleX / 2) / glm::l2Norm(rightDir);
    	realHeight = 2 * glm::l2Norm(upDir);
    	realWidth = 2 * glm::l2Norm(rightDir);
    	pixHeight = realHeight / imageHeight;
    	pixWidth = realWidth / imageWidth;
    }
    glm::vec3 make_camera_ray (float i, float j) {
		glm::vec3 v;
		if (i < imageHeight / 2) {
			v = upDir * (imageHeight - 1 - 2 * i) / imageHeight;
		} else {
			v = - upDir * (2 * i - imageHeight + 1) / imageHeight;
		}
		if (j < imageWidth / 2) {
			v += - rightDir * (imageWidth - 1 - 2 * j) / imageWidth;
		} else {
			v += rightDir * (2 * j - imageWidth + 1) / imageWidth;
		}
		v += viewDir;
		return glm::normalize(v);;
	}
};

// gitHub (tinyobjloader)
std::vector<MyTriangle> parse_triangles (tinyobj::attrib_t attrib,
                                         std::vector<tinyobj::shape_t> shapes,
                                         Params & extremums) {
    std::vector<MyTriangle> res;
    // Loop over shapes
    for (size_t s = 0; s < shapes.size(); s++) {
        // Loop over faces(polygon)
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];
            // Loop over vertices in the face.
            glm::vec3 points[3];
            for (size_t v = 0; v < fv; v++) {
                // access to vertex
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
                tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
                tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];
                points[v] = glm::vec3(vx, vy, vz);
                // find max and min x, y, z
                if (vx > extremums.max[0]) {
                    extremums.max[0] = vx;
                }
                if (vx < extremums.min[0]) { 
                    extremums.min[0] = vx;
                }
                if (vy > extremums.max[1]) {
                    extremums.max[1] = vy;
                }
                if (vy < extremums.min[1]) { 
                    extremums.min[1] = vy;
                }
                if (vz > extremums.max[2]) {
                    extremums.max[2] = vz;
                }
                if (vz < extremums.min[2]) { 
                    extremums.min[2] = vz;
                }
            }
            res.emplace_back(points);
            index_offset += fv;
        }
    }
    return res;
}

void parse_coord (char *filename, WiFiSource &source, Camera &camera) {
    std::ifstream fin;
    fin.open(filename);
    std::string s;
    getline(fin, s);
    std::stringstream ss;
    ss << s;
    ss >> source.p[0] >> source.p[1] >> source.p[2] >> source.power >> source.maxRadius;
    getline(fin, s);
    std::stringstream ss1;
    ss1 << s;
    ss1 >> camera.p[0] >> camera.p[1] >> camera.p[2];
    ss1 >> camera.viewDir[0] >> camera.viewDir[1] >> camera.viewDir[2];
    ss1 >> camera.upDir[0] >> camera.upDir[1] >> camera.upDir[2];
    ss1 >> camera.rightDir[0] >> camera.rightDir[1] >> camera.rightDir[2];
    ss1 >> camera.viewAngleY >> camera.viewAngleX;
    getline(fin, s);
    std::stringstream ss2;
    ss2 << s;
    ss2 >> camera.imageHeight >> camera.imageWidth;
    fin.close();
}

// filtering each plane alone
std::vector<std::vector<std::vector<float>>> box_filter (const std::vector<std::vector<std::vector<float>>> &voxelNetwork) {
	std::vector<std::vector<std::vector<float>>> ans;
	ans.resize(Params::NUM_VOXELS);
    for (int i = 0; i < Params::NUM_VOXELS; ++i) {
        ans[i].resize(Params::NUM_VOXELS);
        for (int j = 0; j < Params::NUM_VOXELS; ++j) {
            ans[i][j].resize(Params::NUM_VOXELS);
        }
    }
#pragma omp parallel for shared(voxelNetwork) schedule(dynamic) num_threads(NUM_THREADS)
	for (int z = 0; z < Params::NUM_VOXELS; ++z) {
#pragma omp parallel for shared(voxelNetwork) schedule(dynamic) num_threads(NUM_THREADS)
		for (int i = 1; i < Params::NUM_VOXELS - 2; ++i) {
#pragma omp parallel for shared(voxelNetwork) schedule(dynamic) num_threads(NUM_THREADS)
			for (int j = 1; j < Params::NUM_VOXELS - 2; ++j) {
				// compute average
				float average = 0.0f;
				for (int ii = -1; ii <= 1; ++ii) {
					for (int jj = -1; jj <= 1; ++jj) {
						average += voxelNetwork[i + ii][j + jj][z];
					}
				}
				ans[i][j][z] = average / 9;
			}
		}
	}
	return ans;
}

Intersection find_nearest_poligon (const glm::vec3 &start, const glm::vec3 &ray,
			  					   const std::vector<MyTriangle> &poligons) {
	Intersection result;
	result.dist = FLT_MAX;
	glm::vec2 curBary;
	float curDist;
	for (const auto &poligon : poligons) {
		if (glm::intersectRayTriangle(start, ray, 
					                  poligon.p[2], poligon.p[1], poligon.p[0],
            	                      curBary, curDist)) {
			// intersection on negative part of the ray
			if (curDist < 0) {
				continue;
			}
			if (curDist < result.dist) {
				result.dist = curDist;
				result.triangle = poligon;
				result.bary = curBary;
			}
		}
	}
	result.get_point();
	result.ray = ray;
	return result;
}

void trace_along_ray (const Intersection &intersection, const glm::vec3 &ray, glm::vec3 start,
					  std::vector<std::vector<std::vector<float>>> &voxels, 
					  float &power, const float &startPower, float sourceMaxRad,
					  glm::vec3 &voxelCoord, const glm::vec3 &startCoord, CoordPointers &cur,
					  float xAdd, glm::vec3 &realCoord, const Params &extremums) {
	//int nStep = intersection.dist * glm::l2Norm(ray) / ray.x / xAdd; 
	//for (int i = 0; i < nStep && power > 0 && 
	for (float dist = 0.0f; dist < intersection.dist && power > EPS &&
						   0 < voxelCoord.x && voxelCoord.x < Params::NUM_VOXELS &&
						   0 < voxelCoord.y && voxelCoord.y < Params::NUM_VOXELS &&
						   0 < voxelCoord.z && voxelCoord.z < Params::NUM_VOXELS;
					//++i) {
					) {
		if (voxels[voxelCoord.x][voxelCoord.y][voxelCoord.z] < power) {
			voxels[voxelCoord.x][voxelCoord.y][voxelCoord.z] = power;
		}
		//if (i == 0) {
		if (dist < EPS) {
			// first iteration with ray
			realCoord += ray;		
		} else {
			*cur.y += (*cur.y - startCoord.y) * xAdd / (*cur.x - startCoord.x);
			*cur.z += (*cur.z - startCoord.z) * xAdd / (*cur.x - startCoord.x);
			*cur.x += xAdd;
		}
		voxelCoord = (realCoord - extremums.min);
		voxelCoord.x = std::floor(voxelCoord.x / extremums.voxelSize.x);
		voxelCoord.y = std::floor(voxelCoord.y / extremums.voxelSize.y);
		voxelCoord.z = std::floor(voxelCoord.z / extremums.voxelSize.z);

		float k = std::min(1.f, intersection.dist / sourceMaxRad);
		power = startPower * (1 - k * k);

		dist = glm::l2Norm(realCoord - start);
	}
}

Intersection trace_ray(const glm::vec3 &start, const glm::vec3 &ray, float startPower, float sourceMaxRad,
					   const std::vector<MyTriangle> &poligons,
					   std::vector<std::vector<std::vector<float>>> &voxels, const Params &extremums) {
	Intersection intersection = find_nearest_poligon(start, ray, poligons);

	glm::vec3 voxelCoord = (start - extremums.min);
	voxelCoord.x = std::floor(voxelCoord.x / extremums.voxelSize.x);
	voxelCoord.y = std::floor(voxelCoord.y / extremums.voxelSize.y);
	voxelCoord.z = std::floor(voxelCoord.z / extremums.voxelSize.z);

	float power = startPower;
	glm::vec3 realCoord = start;
	float xAdd;
	CoordPointers cur;
	glm::vec3 startCoord;

	if (std::fabs(ray.x) > std::fabs(ray.y) && std::fabs(ray.x) > std::fabs(ray.z)) {
		xAdd = extremums.voxelSize.x / 2;
		cur.x = &realCoord.x;
		cur.y = &realCoord.y;
		cur.z = &realCoord.z;
		startCoord.x = start.x;
		startCoord.y = start.y;
		startCoord.z = start.z;
	} else if (std::fabs(ray.y) > std::fabs(ray.x) && std::fabs(ray.y) > std::fabs(ray.z)) {
		xAdd = extremums.voxelSize.y / 2;
		cur.x = &realCoord.y;
		cur.y = &realCoord.x;
		cur.z = &realCoord.z;
		startCoord.x = start.y;
		startCoord.y = start.x;
		startCoord.z = start.z;
	} else {
		xAdd = extremums.voxelSize.z / 2;
		cur.x = &realCoord.z;
		cur.y = &realCoord.y; 
		cur.z = &realCoord.x;
		startCoord.x = start.z;
		startCoord.y = start.y;
		startCoord.z = start.x;
	}
	trace_along_ray(intersection, ray, start, voxels, power, startPower, 
					  sourceMaxRad, voxelCoord, startCoord, cur, xAdd, realCoord, extremums);

    glm::vec3 normal = intersection.normal();
    intersection.ray = glm::reflect(ray, normal);
    intersection.ray = glm::normalize(intersection.ray);
	return intersection;
}

void save_image(const std::vector<std::vector<glm::vec3>> &im, const char *path, Camera &cam) {
    BMP out;
    out.SetSize(cam.imageHeight, cam.imageWidth);

    RGBApixel p;
    p.Alpha = 255;
    for (uint i = 0; i < cam.imageHeight; ++i) {
        for (uint j = 0; j < cam.imageWidth; ++j) {
            p.Red = im[i][j][0];
            p.Green = im[i][j][1];
            p.Blue = im[i][j][2];
            out.SetPixel(j, i, p);
        }
    }

    out.WriteToFile(path);
}

void print_help_msg () {
	std::cout << "Invalid number of command line arguments" << std::endl;
    std::cout << "Input format is: ";
    std::cout << "executable_file *.obj configurations.txt" << std::endl;
    std::cout << "Where in configurations.txt are numbers: " << std::endl;
    std::cout << "3 source of Wi-Fi cordinates, power of the source, max radius of the source - first line" << std::endl;
    std::cout << "3 camera coordinates, 3 view vector coordinates, ";
    std::cout << "3 up vector coordinates, 3 right vector coorsinates, ";
    std::cout << "vertical view angle, horizontal view angle- second line" << std::endl;
    std::cout << "2 numbers - resolution of the output image - third line" << std::endl;
}

void print_voxelnet (const std::vector<std::vector<std::vector<float>>> &voxelNetwork, int n) {
	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < Params::NUM_VOXELS; ++j) {
			for (int k = 0; k < Params::NUM_VOXELS; ++k) {
				std::cout << voxelNetwork[j][k][i] << " ";
			}
			std::cout << std::endl;
		}	
		std::cout << std::endl << std::endl;
	}
}

void print_vec3 (const glm::vec3 &v) {
	std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
}

void print_coord (std::string s, float a, float b, float c) {
	std::cout << s << " " << a << " " << b << " " << c << std::endl;
}

int main (int argc, char **argv) {
    if (argc < 3) {
    	print_help_msg();    
        return 0;
    }
    // load *.obj file, check errors
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err, argv[1]);
    if (!err.empty()) { // `err` may contain warning message.
        std::cerr << err << std::endl;
    }
    if (!ret) {
        exit(1);
    }

    WiFiSource source;
    Camera camera;
    parse_coord(argv[2], source, camera);
    Params extremums;
    // parse attrib into vector of triangles
    std::vector<MyTriangle> poligons = parse_triangles(attrib, shapes, extremums);
   // create voxel net
    std::vector<std::vector<std::vector<float>>> voxelNetwork;
    voxelNetwork.resize(Params::NUM_VOXELS);
    for (int i = 0; i < Params::NUM_VOXELS; ++i) {
        voxelNetwork[i].resize(Params::NUM_VOXELS);
        for (int j = 0; j < Params::NUM_VOXELS; ++j) {
            voxelNetwork[i][j].resize(Params::NUM_VOXELS);
        }
    }
    // calculate size of one voxel
    extremums.voxelSize = (extremums.max - extremums.min) / static_cast<float>(Params::NUM_VOXELS);
    // ray tracing
#pragma omp parallel for shared(source, extremums, poligons, voxelNetwork) schedule(dynamic) num_threads(NUM_THREADS)
    for (int i = 0; i < 200000; ++i) {
        // generate random vec3 with radius = 1
        glm::vec3 randVector = glm::sphericalRand(1.0f);
        Intersection intersection;
        intersection.point = source.p;
        intersection.ray = randVector;
        float power = source.power;
        float maxRad = source.maxRadius;
        while (power > EPS) {
        	//std::cout << power << " ";
        	glm::vec3 prev = intersection.point;
        	intersection = trace_ray(intersection.point, intersection.ray, power, maxRad,
					     			 poligons, voxelNetwork, extremums);
//        	std::cout << glm::l2Norm(intersection.point - prev) << " ";
			float k = std::min(1.f, glm::l2Norm(intersection.point - prev) / maxRad);
			power = power * (1 - k * k);
			maxRad -= glm::l2Norm(intersection.point - prev);
			intersection.point += intersection.ray * 0.01f;

        }
        //std::cout << std::endl;
    }
    // convolve
    voxelNetwork = box_filter(voxelNetwork);
    // create projection
    camera.set_evrthng();
    std::vector<std::vector<glm::vec3>> projection;
    projection.resize(camera.imageHeight);
    for (unsigned int i = 0; i < projection.size(); ++i) {
    	projection[i].resize(camera.imageWidth);
    }

    // generate rays from camera
#pragma omp parallel for shared(camera, poligons, extremums, voxelNetwork) schedule(dynamic) num_threads(NUM_THREADS)
    for (int i = 0; i < static_cast<int>(camera.imageHeight); ++i) {
#pragma omp parallel for shared(camera, poligons, extremums, voxelNetwork) schedule(dynamic) num_threads(NUM_THREADS)
    	for (int j = 0; j < static_cast<int>(camera.imageWidth); ++j) {
    		glm::vec3 ray = camera.make_camera_ray(i, j);
    		float minDist = FLT_MAX, curDist = 0;
        	glm::vec2 baryMin(0.0f), curBary(0.0f);
        	MyTriangle triangle;
			// find nearest poligon, bary - intersect point
			for (auto poligon : poligons) {
				if (glm::intersectRayTriangle(camera.p, ray, 
				                              poligon.p[0], poligon.p[1], poligon.p[2],
            	                              curBary, curDist)) {
            		// intersection on negative part of ray
            		if (curDist < 0) {
            			continue;
            		}
            		// ceiling
            		if (std::fabs(extremums.max.z - poligon.p[0].z) < EPS &&
            			std::fabs(extremums.max.z - poligon.p[1].z) < EPS &&
            			std::fabs(extremums.max.z - poligon.p[2].z) < EPS ) {
            			continue;
            		}
            		if (curDist < minDist) {
            				minDist = curDist;
            				baryMin = curBary;
            				triangle = poligon;
            		}
				}
			}
			// no intersections
			if (std::fabs(FLT_MAX - minDist) > EPS) {
				// turn ray around
				ray *= -1;
				// find intersection point
				glm::vec3 intersection = triangle.p[0] * (1 - baryMin[0] - baryMin[1]) +
										 triangle.p[1] * baryMin[0] + triangle.p[2] * baryMin[1];
				// compute voxel coordinates
				glm::vec3 voxelCoord = intersection - extremums.min;
	        	voxelCoord.x /= extremums.voxelSize.x;
				voxelCoord.y /= extremums.voxelSize.y;
	        	voxelCoord.z /= extremums.voxelSize.z;

	        	if (-1 < voxelCoord.x && voxelCoord.x < Params::NUM_VOXELS + 1 &&
	        		-1 < voxelCoord.y && voxelCoord.y < Params::NUM_VOXELS  + 1 &&
	        		-1 < voxelCoord.z && voxelCoord.z < Params::NUM_VOXELS + 1 ) {
	        		if (triangle.p[0].z < EPS && triangle.p[1].z < EPS && triangle.p[2].z < EPS) {
	        			//floor
	        			projection[i][j] = glm::vec3(1.0f, 1.0f, 1.0f);
	        		} else {
	        			// walls
	        			projection[i][j] = glm::vec3(0.0f, 50.0f, 255.0f);
	        		}
		        	// init power
		        	float pixPower = 0.0f;
		        	// first iteration must be done with ray
    		    	glm::vec3 realCoord = intersection + ray;
    		    	// recompute voxel coordinates
    		    	voxelCoord = realCoord - extremums.min;
    		    	voxelCoord.x /= extremums.voxelSize.x;
					voxelCoord.y /= extremums.voxelSize.y;
					voxelCoord.z /= extremums.voxelSize.z;
					// find absolute max in ray coordinates to iterate it
					float *xCur, *yCur, *zCur;
					float *xStart, *yStart, *zStart;
					float xAdd;																																													
					if (std::fabs(ray[0]) > std::fabs(ray[1])
					    && std::fabs(ray[0]) > std::fabs(ray[2])) {
			        	xAdd = extremums.voxelSize.x / 2;
			        	xCur = &realCoord.x;
			        	yCur = &realCoord.y;
			        	zCur = &realCoord.z;
			        	xStart = &intersection.x;
						yStart = &intersection.y;
						zStart = &intersection.z;
			        } else if (std::fabs(ray[1]) > std::fabs(ray[0])
			                   && std::fabs(ray[1]) > std::fabs(ray[2])) {
			        	xAdd = extremums.voxelSize.y / 2;
			        	xCur = &realCoord.y;
						yCur = &realCoord.x;
						zCur = &realCoord.z;
						xStart = &intersection.y;
						yStart = &intersection.x;
						zStart = &intersection.z;
		    	    } else {
		    	    	xAdd = extremums.voxelSize.z / 2;
		    	    	xCur = &realCoord.z;
						yCur = &realCoord.y;
						zCur = &realCoord.x;
						xStart = &intersection.z;
						yStart = &intersection.y;
						zStart = &intersection.x;
		    	    }
		//  	      float alpha = 0.05f;
		    	    float counter = 0.0f;
		    	    // go along the ray, compute average power
		    	    for (float dist = 0.0f; dist < minDist && 
		    	    						0 < voxelCoord.x && voxelCoord.x < Params::NUM_VOXELS &&
		    	    						0 < voxelCoord.y && voxelCoord.y < Params::NUM_VOXELS &&
		    	    						0 < voxelCoord.z && voxelCoord.z < Params::NUM_VOXELS;) {
	        			// add power
		    	    	pixPower += voxelNetwork[voxelCoord.x][voxelCoord.y][voxelCoord.z] / source.power;
		    	    	counter++;
		//  	      	pixPower *= 1 - alpha;
		//  	      	pixPower += alpha * voxelNetwork[voxelCoord.x][voxelCoord.y][voxelCoord.z];
		 	       		// find new real coords
		        		*yCur += (*yCur - *yStart) * xAdd / (*xCur - *xStart);
		        		*zCur += (*zCur - *zStart) * xAdd / (*xCur - *xStart);
		        		*xCur += xAdd;
		        		// find new voxel coords
				       	voxelCoord = realCoord - extremums.min;
		    	    	voxelCoord.x /= extremums.voxelSize.x;
						voxelCoord.y /= extremums.voxelSize.y;
						voxelCoord.z /= extremums.voxelSize.z;
		        		// find new distance
		        		dist = glm::distance(realCoord, intersection);
		        	}
		        	pixPower /= counter;
		        	pixPower *= (M_E - 1);
		        	pixPower = std::log(1.f + pixPower);
		        	projection[i][j][0] = 255 * pixPower;
		
		            glm::vec3 normal = glm::triangleNormal(triangle.p[0], triangle.p[1], triangle.p[2]);
		            projection[i][j][2] *= std::fabs(glm::dot(normal, ray));
	            } else {
		        	projection[i][j] = glm::vec3(0.0f);
    	    	}
	        }
    	}
    }

    save_image(projection, "out.bmp", camera);


//            glm::vec3 normal = glm::triangleNormal(triangle.p[0], triangle.p[1], triangle.p[2]);
//			vec_dir = glm::reflect(vec_dir, normal);
//			vec_dir = glm::normalize(vec_dir);

    return 0;
}
 