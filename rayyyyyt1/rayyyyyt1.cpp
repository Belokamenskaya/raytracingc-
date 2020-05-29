// rayyyyyt1.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#define _USE_MATH_DEFINES
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include "geometry.h"


struct Light 
{
	Light(const Vec3f& p, const float i) : position(p), intensity(i) {}
	Vec3f position;
	float intensity;
};

struct Material {
	Material(const float r, const Vec4f& a, const Vec3f& color, const float spec) : refractive_index(r), albedo(a), diffuse_color(color), specular_exponent(spec) {}
	Material() : refractive_index(1), albedo(1, 0, 0, 0), diffuse_color(), specular_exponent() {}
	float refractive_index;//показатель преломления 
	Vec4f albedo;
	Vec3f diffuse_color; // дифузный цвет
	float specular_exponent;
};

struct Sphere {
	Vec3f center;
	float radius;
	Material material;

	Sphere(const Vec3f& c, const float r, const Material& m) : center(c), radius(r), material(m) {}
	/*пересекается ли заданный луч (исходящий из orig в направлении dir) с нашей сферой.*/
	bool ray_intersect(const Vec3f& orig, const Vec3f& dir, float& t0) const {
		Vec3f L = center - orig;// this is the vector from point to center
		float tca = L * dir;
		float d2 = L * L - tca * tca;//projection c on the line of ray
		if (d2 > radius * radius) return false;
		// distance from pc to i1
		float thc = sqrtf(radius * radius - d2);
		t0 = tca - thc;
		float t1 = tca + thc;
		if (t0 < 0) t0 = t1;
		if (t0 < 0) return false;// when the sphere is behind the origin p
		return true;
	}
};

Vec3f reflect(const Vec3f & I, const Vec3f & N) {
	return I - N * 2.f * (I * N);
}
/*Snellius*/
Vec3f refract(const Vec3f & I, const Vec3f & N, const float eta_t, const float eta_i = 1.f) { 
	float cosi = -std::max(-1.f, std::min(1.f, I * N));
	if (cosi < 0) return refract(I, -N, eta_i, eta_t); 
	float eta = eta_i / eta_t;
	float k = 1 - eta * eta * (1 - cosi * cosi);
	return k < 0 ? Vec3f(1, 0, 0) : I * eta + N * (eta * cosi - sqrtf(k)); 
}

bool scene_intersect(const Vec3f & orig, const Vec3f & dir, const std::vector<Sphere> & spheres, Vec3f & hit, Vec3f & N, Material & material) {
	float spheres_dist = std::numeric_limits<float>::max();//ставим временно макс значение
	for (size_t i = 0; i < spheres.size(); i++) {//проходим по каждой сфере
		float dist_i;// расстояние от камеры до сферы
		if (spheres[i].ray_intersect(orig, dir, dist_i) && dist_i < spheres_dist) { // если есть пересечение и никакая другая сфера не закрывает текущую
			spheres_dist = dist_i; // меняем расстояние
			hit = orig + dir * dist_i;// строим вектор по точке  и направление*расстояние
			N = (hit - spheres[i].center).normalize(); // нормируем
			material = spheres[i].material;// передаём материал
		}
	}

	float checkerboard_dist = std::numeric_limits<float>::max();
	if (fabs(dir.y) > 1e-3) {
	float d = -(orig.y +4) / dir.y; //y=-4
	Vec3f pt = orig + dir * d;
	if (d > 0 && fabs(pt.x) < 10 && pt.z<-10 && pt.z>-30 && d < spheres_dist) {
		checkerboard_dist = d;
		hit = pt;
		N = Vec3f(0, 1, 0);
		material.diffuse_color =  Vec3f(1, .7, .3);
		material.diffuse_color = material.diffuse_color * .3;
	}
}
	
	
	return std::min(spheres_dist, checkerboard_dist) < 1000;
}

Vec3f cast_ray(const Vec3f & orig, const Vec3f & dir, const std::vector<Sphere> & spheres, const std::vector<Light> & lights, size_t depth = 0) {
	Vec3f point; //вектор от камеры до пересечения со сценой 
	Vec3f N; //  нормальный вектор в точке 
	Material material;
	
	if (depth > 4 || !scene_intersect(orig, dir, spheres, point, N, material)) {
		return Vec3f(0.2, 0.7, 0.8); // background color
	}
	
	Vec3f reflect_dir = reflect(dir, N).normalize();
	Vec3f refract_dir = refract(dir, N, material.refractive_index).normalize();
	Vec3f reflect_orig = reflect_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3; // offset the original point to avoid occlusion by the object itself
	Vec3f refract_orig = refract_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3;
	Vec3f reflect_color = cast_ray(reflect_orig, reflect_dir, spheres, lights, depth + 1);
	Vec3f refract_color = cast_ray(refract_orig, refract_dir, spheres, lights, depth + 1);

	float diffuse_light_intensity = 0, specular_light_intensity = 0;
	for (size_t i = 0; i < lights.size(); i++) {
		Vec3f light_dir = (lights[i].position - point).normalize();//вектор описывающий направление света
		float light_distance = (lights[i].position - point).norm(); //расстояние от источника до камеры
		/*убеждаемся, не пересекает ли луч точка-источник света объекты нашей сцены, и если пересекает, то пропускам текущий источник света.
		 сдвигаю точку в направлении нормали:*/
		Vec3f shadow_orig = light_dir * N < 0 ? point - N * 1e-3 : point + N * 1e-3; 
		Vec3f shadow_pt, shadow_N;
		Material tmpmaterial;
		if (scene_intersect(shadow_orig, light_dir, spheres, shadow_pt, shadow_N, tmpmaterial) && (shadow_pt - shadow_orig).norm()+54 < light_distance)
			continue;

		diffuse_light_intensity += lights[i].intensity * std::max(0.f, light_dir * N);
		specular_light_intensity += powf(std::max(0.f, reflect(light_dir, N) * dir), material.specular_exponent) * lights[i].intensity;
	}
	return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + Vec3f(1., 1., 1.) * specular_light_intensity * material.albedo[1] + reflect_color * material.albedo[2] + refract_color * material.albedo[3];
}

void render(const std::vector<Sphere> & spheres, const std::vector<Light> & lights) {
	const int   width = 300;
	const int   height = 300;
	const float fov = M_PI / 3.;
	std::vector<Vec3f> framebuffer(width * height);

#pragma omp parallel for
	for (size_t j = 0; j < height; j++) { 
		for (size_t i = 0; i < width; i++) {
			float dir_x = (i + 0.5) - width / 2.;
			float dir_y = -(j + 0.5) + height / 2.;   
			float dir_z = -height / (2. * tan(fov / 2.));
			framebuffer[i + j * width] = cast_ray(Vec3f(0, 0, 0), Vec3f(dir_x, dir_y, dir_z).normalize(), spheres, lights);
		}
	}

	std::ofstream ofs; // save the framebuffer to file
	ofs.open("./out.ppm", std::ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
	for (size_t i = 0; i < height * width; ++i) {
		
		for (size_t j = 0; j < 3; j++) {
			ofs << (char)(255 * std::max(0.f, std::min(1.f, framebuffer[i][j])));
		}
	}
	ofs.close();
}

int main() {
	Material      ivory(1.0, Vec4f(0.6, 0.3, 0.1, 0.0), Vec3f(0.4, 0.4, 0.3), 50.);
	Material      glass(1.5, Vec4f(0.0, 0.5, 0.1, 0.8), Vec3f(0.6, 0.7, 0.8), 125.);
	Material red_rubber(4.0, Vec4f(0.9, 0.1, 0.0, 0.0), Vec3f(0.3, 0.1, 0.1), 10.);
	Material     mirror(1.0, Vec4f(0.0, 10.0, 0.8, 0.0), Vec3f(1.0, 1.0, 1.0), 1425.);
	
	

	 std::vector<Sphere> spheres;
	//spheres.push_back(Sphere(Vec3f(-3, 0, -16), 2, ivory));
	//spheres.push_back(Sphere(Vec3f(-1.0, -1.5, -12), 2, glass));
	spheres.push_back(Sphere(Vec3f(1.5, -0.5, -18), 3, red_rubber));
	spheres.push_back(Sphere(Vec3f(7, 5, -18), 4, mirror));

	std::vector<Light>  lights;
	lights.push_back(Light(Vec3f(-20, 20, 20), 1.5));
	lights.push_back(Light(Vec3f(30, 50, -25), 1.8));
	lights.push_back(Light(Vec3f(30, 20, 30), 1.7));

	


	render(spheres, lights);
	system("pause");
	return 0;
}

