
#include <algorithm>
#include <iostream>
#include <cstdlib> // drand48
#include <limits> // numeric_limits
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

#include "vec3.h"
#include "Material.h"
#include "Scene.h"
#include "Camera.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

unsigned char* framebufferToArray(std::vector<std::vector<vec3> > &framebuffer)
{
    int width = (int)framebuffer.size();
    int height = (int)framebuffer[0].size();

    unsigned char *output = new unsigned char[width * height * 3];
    unsigned char *output_ptr = output;

    for (int j = height - 1; j >= 0; --j)
    {
        unsigned char *temp = new unsigned char[3 * width];
        unsigned char *temp_ptr = temp;

        for (int i = 0; i < width; ++i)
        {
            int rgb[3];
            rgb[0] = (unsigned char)(255.99 * framebuffer[i][j].x());
            rgb[1] = (unsigned char)(255.99 * framebuffer[i][j].y());
            rgb[2] = (unsigned char)(255.99 * framebuffer[i][j].z());
            std::copy(rgb, rgb + 3, temp_ptr);
            temp_ptr += 3;
        }

        std::copy(temp, temp + (width * 3), output_ptr);
        output_ptr += width * 3;
        delete[] temp;
    }

    return output;
}

inline double fastMin(double l, double r)
{
    return l < r ? l : r;
}

inline double fastMax(double l, double r)
{
    return l > r ? l : r;
}

// recursively compute the color of a pixel given a starting ray
vec3 color(const Ray &r, Scene *scene, int depth)
{
    HitRecord hit_record;
    if (scene->hit(r, 0.001, std::numeric_limits<double>::max(), hit_record))
    {
        Ray scattered;
        vec3 attenuation;

        // limit the call stack. only proceed when successfully scattered.
        if (depth < 50 && hit_record.material->scatter(r, hit_record, attenuation, scattered))
            return attenuation * color(scattered, scene, depth + 1);
        else
            return vec3(0.0);
    }
    else
    {
        // color the backdrop (arbitrary)
        vec3 dir = createUnitVector(r.direction());
        double t = 0.5 * (dir.y() + 1.0);
        return lerp(vec3(1.0), vec3(0.5, 0.7, 1.0), t);
    }
}

Scene* createRandomScene(int num_entities)
{
    Scene *scene = new Scene();
    scene->entities.push_back(new Sphere(vec3(0.0, -1000.0, 0.0), 1000.0, new Lambertian(vec3(0.5)) ) );

    double dim = sqrt(num_entities) / 2.0;
    for (double a = -dim; a < dim; ++a)
    for (double b = -dim; b < dim; ++b)
    {
        double curr_material = drand48();
        vec3 center(a + 0.9 * drand48(), 0.2, b + 0.9 * drand48());
        if ((center - vec3(4.0, 0.2, 0.0)).mag() > 0.9)
        {
            if (curr_material < 0.8)
                scene->entities.push_back(new Sphere(center, 0.2, new Lambertian(vec3(drand48() * drand48(), drand48() * drand48(), drand48() * drand48()))));
            else if (curr_material < 0.95)
                scene->entities.push_back(new Sphere(center, 0.2, new Metallic(0.5 * vec3(1.0 + drand48(), 1.0 + drand48(), drand48()), 0.5 * drand48())));
            else
                scene->entities.push_back(new Sphere(center, 0.2, new Dielectric(vec3(1.0), 1.5)));
        }
    }

    scene->entities.push_back(new Sphere(vec3(0.0, 1.0, 0.0), 1.0, new Dielectric(vec3(1.0), 1.5)));
    scene->entities.push_back(new Sphere(vec3(-4.0, 1.0, 0.0), 1.0, new Lambertian(vec3(0.4, 0.2, 0.1))));
    scene->entities.push_back(new Sphere(vec3(4.0, 1.0, 0.0), 1.0, new Metallic(vec3(0.7, 0.6, 0.5), 0.0)));

    return scene;
}

struct Settings
{
    bool use_antialiasing;
    bool use_gamma_correction;
    int width;
    int height;
    int num_aa_samples;
};

// threads loop through the frame in an interleaving fashion. each computes and stores RGB vec3 to global framebuffer.
void render(Scene *scene, Camera &camera, Settings &settings, int thread_id, int num_threads, std::vector<std::vector<vec3> > &framebuffer)
{
    for (int idx = thread_id; idx < settings.width * settings.height; idx += num_threads)
    {
        int i = idx % settings.width;
        int j = idx / settings.width;
        vec3 col(0.0);

        if (settings.use_antialiasing)
        {
            // apply stochastic sampling approach over evenly spaced sub pixel box regions
            double delta_u = 1.0 / double(settings.width);
            double delta_v = 1.0 / double(settings.height);
            double sub_pixel_dimension = sqrt(settings.num_aa_samples);

            for (double s_x = 0.0; s_x < sub_pixel_dimension; ++s_x)
            {
                for (double s_y = 0.0; s_y < sub_pixel_dimension; ++s_y)
                {
                    // pixel offset + sub-pixel offset + random point in sub-pixel box
                    double u = (double(i) / double(settings.width)) + (delta_u / sub_pixel_dimension) * (s_x * drand48());
                    double v = (double(j) / double(settings.height)) + (delta_v / sub_pixel_dimension) * (s_y * drand48());
                    col += color(camera.getRay(u, v), scene, 0);
                }
            }

            // could apply gaussian filter here. but choosing simple box filter (equal weighting)
            col /= settings.num_aa_samples;
        }
        else
        {
            double u = double(i) / double(settings.width);
            double v = double(j) / double(settings.height);
            col = color(camera.getRay(u, v), scene, 0);
        }

        if (settings.use_gamma_correction)
            col = vec3(sqrt(col.x()), sqrt(col.y()), sqrt(col.z())); // raise to power of 1/X, where I use X = 2

        framebuffer[i][j] = vec3(col);

        std::cout << double(idx) / double(settings.width * settings.height) * 100 << "%\n";
    }
}

int main(int argc, char **argv)
{
    Settings settings;
    settings.use_antialiasing = true;
    settings.use_gamma_correction = true;
    settings.width = 720;
    settings.height = 480;
    settings.num_aa_samples = 36;

    // setup entities and scene
    Scene *scene = createRandomScene(50);

    vec3 camera_center(5.5, 2.0, 1.0);
    vec3 look_at(0.0, 0.0, -1.0);
    double distance_to_focus = (camera_center - look_at).mag();
    double aperture = 0.02;

    Camera camera(camera_center, look_at, vec3(0.0, 1.0, 0.0), 90.0,
                  settings.width / settings.height, aperture, distance_to_focus);

    unsigned int num_threads = std::thread::hardware_concurrency();
    std::vector<std::thread> threads;

    std::vector<std::vector<vec3> > framebuffer(settings.width);
    for (int i = 0; i < settings.width; ++i)
        framebuffer[i].resize(settings.height);

    // begin execution
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_threads; ++i)
        threads.emplace_back(std::thread(render, scene, std::ref(camera),
                             std::ref(settings), i, num_threads, std::ref(framebuffer)));

    for (auto &t : threads)
        t.join();

    // end execution
    auto curr_time = std::chrono::high_resolution_clock::now();
    float time = std::chrono::duration_cast<std::chrono::milliseconds>(curr_time - start_time).count() / 1000.0f;
    std::cout << "\n\n Execution time: " << time << " (seconds)\n\n";

    // output to file
    unsigned char *output_image = framebufferToArray(framebuffer);
    if (stbi_write_png("image.png", settings.width, settings.height, 3, output_image, 3 * settings.width) == 0)
        std::cout << "stb_image failed to write to image.\n";

    delete scene;
    delete[] output_image;
}
