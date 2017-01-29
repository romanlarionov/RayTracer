
#include <cstring>
#include <algorithm>
#include <iostream>
#include <limits> // numeric_limits
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>

#include "vec3.h"
#include "Material.h"
#include "Scene.h"
#include "Camera.h"
#include "Texture.h"
#include "Utilities.h"

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
    std::vector<Entity *> entities;

    double dim = sqrt(num_entities) / 2.0;
    for (int a = 0; a < sqrt(num_entities); ++a)
    for (int b = 0; b < sqrt(num_entities); ++b)
    {
		double curr_material = getUnitRandom();
        vec3 center(a - dim + 0.9 * getUnitRandom(), 0.2, b - dim + 0.9 * getUnitRandom());
        if ((center - vec3(4.0, 0.2, 0.0)).mag() > 0.9)
        {
            if (curr_material < 0.8)
            {
                ConstantTexture *texture = new ConstantTexture(vec3(getUnitRandom() * getUnitRandom(), getUnitRandom() * getUnitRandom(), getUnitRandom() * getUnitRandom()));
                entities.push_back(new Sphere(center, 0.2, new Lambertian(texture)));
            }
            else if (curr_material < 0.95)
            {
                ConstantTexture *texture = new ConstantTexture(0.5 * vec3(1.0 + getUnitRandom(), 1.0 + getUnitRandom(), getUnitRandom()));
                entities.push_back(new Sphere(center, 0.2, new Metallic(texture, 0.5 * getUnitRandom())));
            }
            else
                entities.push_back(new Sphere(center, 0.2, new Dielectric(new ConstantTexture(vec3(1.0)), 1.5)));
        }
    }

    entities.push_back(new Sphere(vec3(0.0, -1000.0, 0.0), 1000.0, new Lambertian(new CheckerTexture(new ConstantTexture(vec3(0.0)), new ConstantTexture(vec3(1.0))) )));
    entities.push_back(new Sphere(vec3(0.0, 1.0, 0.0), 1.0, new Dielectric(new ConstantTexture(vec3(1.0)), 1.5)));
    entities.push_back(new Sphere(vec3(-4.0, 1.0, 0.0), 1.0, new Lambertian(new ConstantTexture(vec3(0.4, 0.2, 0.1)))));
    entities.push_back(new Sphere(vec3(4.0, 1.0, 0.0), 1.0, new Metallic(new ConstantTexture(vec3(0.7, 0.6, 0.5)), 0.0)));

    return new Scene(entities);
}

struct Settings
{
    bool use_antialiasing;
    bool use_gamma_correction;
    bool print_output;
    int width;
    int height;
    int num_aa_samples;
    std::mutex mtx;
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
                    double u = (double(i) / double(settings.width)) + (delta_u / sub_pixel_dimension) * (s_x * getUnitRandom());
                    double v = (double(j) / double(settings.height)) + (delta_v / sub_pixel_dimension) * (s_y * getUnitRandom());
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

        if (settings.print_output)
        {
            std::lock_guard<std::mutex> l(settings.mtx);
            std::cout << "\r" << (idx * 100) / (settings.width * settings.height)<< "%" << std::flush;
        }
    }
}

int main(int argc, char **argv)
{
    Settings settings;
    settings.use_antialiasing = true;
    settings.use_gamma_correction = true;
    settings.print_output = false;
    settings.width = 720;
    settings.height = 480;
    settings.num_aa_samples = 100;

    // command line input
    if (argc > 1)
    {
        if (strcmp(argv[1], "-p") == 0)
        {
            settings.print_output = true;
        }
    }

    // setup entities and scene
    Scene *scene = createRandomScene(500);

    vec3 camera_center(5.5, 1.0, 3.0);
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

    for (unsigned int i = 0; i < num_threads; ++i)
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
    if (stbi_write_png("images/current.png", settings.width, settings.height, 3, output_image, 3 * settings.width) == 0)
        std::cout << "stb_image failed to write to image.\n";

    delete scene;
    delete[] output_image;
}
