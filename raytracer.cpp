
#include <cstring>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <limits> // numeric_limits
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>
#include <mutex>

#include "vec3.h"
#include "Material.h"
#include "BVH.h"
#include "Scene.h"
#include "Camera.h"
#include "Texture.h"
#include "Utilities.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

unsigned char* framebufferToArray(std::vector<std::vector<vec3> > &framebuffer)
{
    int width = static_cast<int>(framebuffer.size());
    int height = static_cast<int>(framebuffer[0].size());

    unsigned char *output = new unsigned char[width * height * 3];
    unsigned char *output_ptr = output;

    for (int j = height - 1; j >= 0; --j)
    {
        unsigned char *temp = new unsigned char[3 * width];
        unsigned char *temp_ptr = temp;

        for (int i = width - 1; i > 0; --i)
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

    // Light
    entities.push_back(new Sphere(vec3(0.0, 1000.0, 0.0), 1000.0, new DiffuseLight(new ConstantTexture(vec3(0.5)))));

    entities.push_back(new Sphere(vec3(0.0, -1000.0, 0.0), 1000.0, new Lambertian(new CheckerTexture(new ConstantTexture(vec3(0.0)), new ConstantTexture(vec3(1.0))) )));
    entities.push_back(new Sphere(vec3(0.0, 1.0, 0.0), 1.0, new Dielectric(new ConstantTexture(vec3(1.0)), 1.5)));
    entities.push_back(new Sphere(vec3(-4.0, 1.0, 0.0), 1.0, new Lambertian(new ConstantTexture(vec3(0.4, 0.2, 0.1)))));
    entities.push_back(new Sphere(vec3(4.0, 1.0, 0.0), 1.0, new Metallic(new ConstantTexture(vec3(0.7, 0.6, 0.5)), 0.0)));

    return new Scene(new BVHNode(entities.data(), static_cast<int>(entities.size())));
}

Scene* createCornellBox()
{
    std::vector<Entity *> entities;
    Material *red   = new Lambertian(new ConstantTexture(vec3(0.65, 0.05, 0.05)));
    Material *white = new Lambertian(new ConstantTexture(vec3(0.73, 0.73, 0.73)));
    Material *green = new Lambertian(new ConstantTexture(vec3(0.12, 0.45, 0.15)));
    Material *light = new DiffuseLight(new ConstantTexture(vec3(15.0)));

    // Walls
    entities.emplace_back(new yz_Rect(0, 10, 0, 10, 10, vec3(-1.0, 0.0, 0.0), green));
    entities.emplace_back(new yz_Rect(0, 10, 0, 10,  0, vec3(1.0, 0.0, 0.0), red));
    entities.emplace_back(new xz_Rect(0, 10, 0, 10, 10, vec3(0.0, -1.0, 0.0), white));
    entities.emplace_back(new xz_Rect(0, 10, 0, 10,  0, vec3(0.0, 1.0, 0.0), white));
    entities.emplace_back(new xy_Rect(0, 10, 0, 10, 10, vec3(0.0, 0.0, -1.0), white));
    entities.emplace_back(new xz_Rect(4, 6, 4, 6, 9.9999, vec3(0.0, -1.0, 0.0), light));

    // Objects
    entities.emplace_back(new AxisAlignedBox(vec3(4.82, 0, 1.2), vec3(7.82, 3.0, 4.18), white));
    entities.emplace_back(new AxisAlignedBox(vec3(2.36, 0, 5.36), vec3(5.36, 6.0, 8.36), white));
    entities.push_back(new Sphere(vec3(2.0, 1.0001, 1.5), 1.0, new Dielectric(new ConstantTexture(vec3(1.0, 1.0, 1.0)), 1.5)));

    return new Scene(new EntityList(entities));
}

struct Settings
{
    bool use_gamma_correction;
    bool print_output;
    int width;
    int height;
    int num_aa_samples;
    int stack_depth;
    std::mutex mtx;
};

// recursively compute the color of a pixel given a starting ray
vec3 color(const Ray &r, Scene *scene, int depth, const int max_depth)
{
    HitRecord hit_record;
    if (scene->hit(r, 0.001, std::numeric_limits<double>::max(), hit_record))
    {
        Ray scattered;
        vec3 attenuation;
        vec3 emitted = hit_record.material->emitted(hit_record.u, hit_record.v, hit_record.position);

        // limit the call stack. only proceed when successfully scattered.
        if (depth < max_depth && hit_record.material->scatter(r, hit_record, attenuation, scattered))
            return emitted + attenuation * color(scattered, scene, depth + 1, max_depth);

        return emitted;
    }

    return vec3(0.0);
}

// threads loop through the frame in an interleaving fashion. each computes and stores RGB vec3 to global framebuffer.
void render(Scene *scene, Camera &camera, Settings &settings, int thread_id, int num_threads, std::vector<std::vector<vec3> > &framebuffer)
{
    for (int idx = thread_id; idx < settings.width * settings.height; idx += num_threads)
    {
        int i = idx % settings.width;
        int j = idx / settings.width;
        vec3 col(0.0);

        // apply jittered sampling approach over evenly spaced sub pixel box regions
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
                vec3 temp_color = color(camera.getRay(u, v), scene, 0, settings.stack_depth);
                if (!(temp_color[0] == temp_color[0]) || !(temp_color[1] == temp_color[1]) || !(temp_color[2] == temp_color[2]))
                    temp_color = vec3(0.0);
                col += temp_color;
            }
        }

        // could apply gaussian filter here. but choosing simple box filter (equal weighting)
        col /= settings.num_aa_samples;

        if (settings.use_gamma_correction)
            col = vec3(sqrt(col.x()), sqrt(col.y()), sqrt(col.z())); // raise to power of 1/X, where I use X = 2

        // todo: need to implement high dynamic range for this to become unnecessary.
        if (col.x() > 1.0) col[0] = 1.0;
        if (col.y() > 1.0) col[1] = 1.0;
        if (col.z() > 1.0) col[2] = 1.0;

        framebuffer[i][j] = col;

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
    settings.use_gamma_correction = true;
    settings.print_output = false;
    settings.width = 720;
    settings.height = 720;
    settings.num_aa_samples = 2025;
    settings.stack_depth = 100;

    // command line input
    if (argc > 1)
    {
        if (strcmp(argv[1], "-p") == 0)
        {
            settings.print_output = true;
        }
    }

    // setup entities and scene
    //Scene *scene = createRandomScene(0);
    Scene *scene = createCornellBox();

    vec3 camera_center(5.0, 5.0, -10.0);
    vec3 look_at(5.0, 5.0, 0.0);
    double distance_to_focus = (look_at - camera_center).mag();
    double aperture = 0.0;
    double fov = 50.0;
    
    Camera camera(camera_center, look_at, vec3(0.0, 1.0, 0.0), fov,
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
    float time = std::chrono::duration_cast<std::chrono::seconds>(curr_time - start_time).count();

    int seconds = (int)time % 60;
    int minutes = (int)time / 60;
    int hours   = hours / 60;
    std::ofstream output("images/render_info.txt");
    output << "Execution time: " << hours << "h, " << minutes << "m, " << seconds << "s\n";
    output << "Samples per pixel: " << settings.num_aa_samples << "\n";
    output << "Used gamma correction: " << settings.use_gamma_correction << "\n";
    output.close();

    // output to file
    unsigned char *output_image = framebufferToArray(framebuffer);
    if (stbi_write_png("images/current.png", settings.width, settings.height, 3, output_image, 3 * settings.width) == 0)
        std::cout << "stb_image failed to write to image.\n";

    delete scene;
    delete[] output_image;
}