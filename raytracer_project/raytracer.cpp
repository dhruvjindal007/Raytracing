#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <algorithm> // std::clamp
#include <chrono>

using namespace std;

const double EPSILON = 1e-4;
const double PI = 3.141592653589793;
const int    MAX_DEPTH = 5;

// GLSL-style mix
inline double mix(const double &a, const double &b, const double &t) {
    return b * t + a * (1.0 - t);
}

struct Vec3 {
    double x, y, z;
    Vec3(double v = 0) : x(v), y(v), z(v) {}
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3& operator+=(const Vec3& b) { x += b.x; y += b.y; z += b.z; return *this; }
    Vec3 operator+(const Vec3 &b) const { return Vec3(x+b.x, y+b.y, z+b.z); }
    Vec3 operator-(const Vec3 &b) const { return Vec3(x-b.x, y-b.y, z-b.z); }
    Vec3 operator*(double b)     const { return Vec3(x*b, y*b, z*b); }
    Vec3 operator*(const Vec3 &b)const { return Vec3(x*b.x, y*b.y, z*b.z); } // Hadamard
    Vec3 operator/(double b)     const { return Vec3(x/b, y/b, z/b); }
    Vec3 operator-()       const { return Vec3(-x, -y, -z); }

    Vec3& normalize() {
        double len2 = x*x + y*y + z*z;
        if (len2 > 0) {
            double inv = 1.0 / std::sqrt(len2);
            x *= inv; y *= inv; z *= inv;
        }
        return *this;
    }
    double length()   const { return std::sqrt(x*x + y*y + z*z); }
    double dot(const Vec3 &b) const { return x*b.x + y*b.y + z*b.z; }
    Vec3 cross(const Vec3 &b) const {
        return Vec3(y*b.z - z*b.y, z*b.x - x*b.z, x*b.y - y*b.x);
    }
};

struct Ray {
    Vec3 orig, dir;
    Ray(const Vec3 &o, const Vec3 &d) : orig(o), dir(d) {}
};

struct Sphere {
    Vec3   center;
    double radius, radius2;
    Vec3   surfaceColor, emissionColor;
    double transparency, reflection;

    Sphere(const Vec3 &c, double r, const Vec3 &sc, double refl=0.0, double transp=0.0, const Vec3 &ec=0.0)
        : center(c), radius(r), radius2(r*r),
          surfaceColor(sc), emissionColor(ec),
          transparency(transp), reflection(refl) {}

    bool isLight() const {
        return (emissionColor.x > 0) || (emissionColor.y > 0) || (emissionColor.z > 0);
    }

    bool intersect(const Ray &ray, double &t0, double &t1) const {
        // Geometric solution
        Vec3 L = center - ray.orig;
        double tca = L.dot(ray.dir);
        if (tca < 0) return false;
        double d2 = L.dot(L) - tca*tca;
        if (d2 > radius2) return false;
        double thc = std::sqrt(std::max(0.0, radius2 - d2));
        t0 = tca - thc;
        t1 = tca + thc;
        return true;
    }
};

// Fresnel (Schlick or exact). Using exact here.
static inline void fresnel(const Vec3 &I, const Vec3 &N, double ior, double &kr) {
    double cosi = std::clamp(I.dot(N), -1.0, 1.0);
    double etai = 1.0, etat = ior;
    if (cosi > 0) std::swap(etai, etat);
    // Snell
    double sint = etai / etat * std::sqrt(std::max(0.0, 1.0 - cosi*cosi));
    if (sint >= 1.0) {
        kr = 1.0; // Total internal reflection
    } else {
        double cost = std::sqrt(std::max(0.0, 1.0 - sint*sint));
        cosi = std::fabs(cosi);
        double Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        double Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        kr = (Rs*Rs + Rp*Rp) * 0.5;
    }
}

// Safe refraction (returns zero vector on TIR)
static inline Vec3 refract(const Vec3 &I, const Vec3 &N, double ior) {
    double cosi = std::clamp(I.dot(N), -1.0, 1.0);
    double etai = 1.0, etat = ior;
    Vec3 n = N;
    if (cosi < 0) {
        cosi = -cosi;
    } else {
        std::swap(etai, etat);
        n = -n;
    }
    double eta = etai / etat;
    double k = 1.0 - eta*eta*(1.0 - cosi*cosi);
    if (k < 0.0) return Vec3(0.0); // TIR
    return I*eta + n*(eta*cosi - std::sqrt(k));
}

static Vec3 trace(const Ray &ray, const std::vector<Sphere> &spheres, int depth) {
    if (depth > MAX_DEPTH) return Vec3(0.0);

    const Sphere* hit = nullptr;
    double tnear = std::numeric_limits<double>::infinity();

    // Find closest hit
    for (const auto &s : spheres) {
        double t0 = 0, t1 = 0;
        if (s.intersect(ray, t0, t1)) {
            if (t0 < 0) t0 = t1;
            if (t0 >= 0 && t0 < tnear) {
                tnear = t0;
                hit = &s;
            }
        }
    }

    // Miss => background
    if (!hit) return Vec3(0.2, 0.7, 0.8);

    // Hit point & normal
    Vec3 phit = ray.orig + ray.dir * tnear;
    Vec3 nhit = (phit - hit->center);
    nhit.normalize();

    // Flip normal if we’re inside
    bool inside = false;
    if (ray.dir.dot(nhit) > 0) {
        nhit = -nhit;
        inside = true;
    }

    const double bias = EPSILON;
    Vec3 surfaceColor(0.0);

    // Mirror / glass
    if ((hit->transparency > 0.0 || hit->reflection > 0.0) && depth < MAX_DEPTH) {
        // Fresnel
        double kr;
        fresnel(ray.dir, nhit, 1.1 /*IOR*/, kr);
        // Reflection
        Vec3 reflDir = (ray.dir - nhit * 2.0 * ray.dir.dot(nhit)).normalize();
        Vec3 reflCol = trace(Ray(phit + nhit * bias, reflDir), spheres, depth + 1);

        // Refraction (if any)
        Vec3 refrCol(0.0);
        if (hit->transparency > 0.0) {
            Vec3 refrDir = refract(ray.dir, nhit, 1.1).normalize();
            if (refrDir.length() > 0.0) {
                refrCol = trace(Ray(phit - nhit * bias, refrDir), spheres, depth + 1);
            }
        }

        // Combine (note: hit->reflection is not strictly used here; you can blend it in if you like)
        surfaceColor = (reflCol * kr + refrCol * (1.0 - kr) * hit->transparency) * hit->surfaceColor;
    } else {
        // Diffuse shading with one point light (the light is the emitting sphere in the list)
        for (const auto &light : spheres) {
            if (!light.isLight()) continue;

            Vec3 toLight = (light.center - phit);
            double distToLight = toLight.length();
            Vec3 lightDir = toLight / distToLight; // normalized

            // Shadow ray: skip self and skip light sphere
            bool shadow = false;
            for (const auto &obj : spheres) {
                if (&obj == hit || obj.isLight()) continue;
                double t0, t1;
                if (obj.intersect(Ray(phit + nhit * bias, lightDir), t0, t1)) {
                    // If t0 is positive and less than distance to light, we’re blocked
                    if (t0 > 0.0 && t0 < distToLight) {
                        shadow = true;
                        break;
                    }
                }
            }

            if (!shadow) {
                double NdotL = std::max(0.0, nhit.dot(lightDir));
                // Simple Lambert + light intensity from emissionColor
                surfaceColor += hit->surfaceColor * NdotL * light.emissionColor;
            }
        }
    }

    // Ambient term
    surfaceColor = surfaceColor + hit->surfaceColor * 0.1;

    // Return
    return surfaceColor;
}

static void render(const vector<Sphere> &spheres) {
    const unsigned width = 640, height = 480;
    vector<Vec3> image(width * height);

    const double invW = 1.0 / double(width);
    const double invH = 1.0 / double(height);
    const double fov = 30.0;
    const double aspect = double(width) / double(height);
    const double angle = std::tan(PI * 0.5 * fov / 180.0);

    for (unsigned y = 0; y < height; ++y) {
        for (unsigned x = 0; x < width; ++x) {
            double xx = (2.0 * ((x + 0.5) * invW) - 1.0) * angle * aspect;
            double yy = (1.0 - 2.0 * ((y + 0.5) * invH)) * angle;
            Vec3 dir(xx, yy, -1.0);
            dir.normalize();
            image[y * width + x] = trace(Ray(Vec3(0.0), dir), spheres, 0);
        }
    }

    // Gamma correct & clamp to 8-bit
    auto toByte = [](double c) -> unsigned char {
        c = std::clamp(c, 0.0, 1.0);
        // gamma 2.2
        c = std::pow(c, 1.0 / 2.2);
        return static_cast<unsigned char>(c * 255.0 + 0.5);
    };

    ofstream ofs("./image.ppm", ios::out | ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (const auto &p : image) {
        ofs << toByte(p.x) << toByte(p.y) << toByte(p.z);
    }
    ofs.close();
}

int main() {
    srand(13);

    vector<Sphere> spheres;
    // Ground
    spheres.emplace_back(Vec3(0, -10004, -20), 10000, Vec3(0.20, 0.20, 0.20));
    // Objects
    spheres.emplace_back(Vec3(0,    0, -20), 4, Vec3(1.00, 0.32, 0.36), 1.0, 0.5);
    spheres.emplace_back(Vec3(5,   -1, -15), 2, Vec3(0.90, 0.76, 0.46), 1.0, 0.0);
    spheres.emplace_back(Vec3(5,    0, -25), 3, Vec3(0.65, 0.77, 0.97), 1.0, 0.0);
    spheres.emplace_back(Vec3(-5.5, 0, -15), 3, Vec3(0.90, 0.90, 0.90), 1.0, 0.0);
    // Light (emissive sphere)
    spheres.emplace_back(Vec3(0, 20, -30), 3, Vec3(0), 0.0, 0.0, Vec3(3)); // emission = intensity

    auto start = std::chrono::high_resolution_clock::now();
    render(spheres);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    std::cout << "Render completed in " << elapsed.count() << " seconds.\n";
    return 0;
}
