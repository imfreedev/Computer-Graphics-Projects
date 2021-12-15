//
// Implementation for Yocto/RayTrace.
//

//
// LICENSE:
//
// Copyright (c) 2016 -- 2021 Fabio Pellacini
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "yocto_raytrace.h"

#include <math.h>
#include <yocto/yocto_cli.h>
#include <yocto/yocto_geometry.h>
#include <yocto/yocto_noise.h>  //added
#include <yocto/yocto_parallel.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shading.h>
#include <yocto/yocto_shape.h>

#include <iostream>

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR SCENE EVALUATION
// -----------------------------------------------------------------------------
namespace yocto {

// Generates a ray from a camera for yimg::image plane coordinate uv and
// the lens coordinates luv.
static ray3f eval_camera(const camera_data& camera, const vec2f& uv) {
  auto film = camera.aspect >= 1
                  ? vec2f{camera.film, camera.film / camera.aspect}
                  : vec2f{camera.film * camera.aspect, camera.film};
  auto q    = transform_point(camera.frame,
      {film.x * (0.5f - uv.x), film.y * (uv.y - 0.5f), camera.lens});
  auto e    = transform_point(camera.frame, {0, 0, 0});
  return {e, normalize(e - q)};
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR PATH TRACING
// -----------------------------------------------------------------------------
namespace yocto {

// Matte renderer.
static vec4f shade_matte(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  // YOUR CODE GOES HERE ----
  return {0, 0, 0, 0};
}

// Eyelight renderer.
static vec4f shade_eyelight(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }
  const auto& material = scene.materials[isec.instance];
  const auto& shape    = scene.shapes[isec.instance];
  const auto& normal   = eval_normal(shape, isec.element, isec.uv);
  vec4f       c = {material.color.x, material.color.y, material.color.z, 0};
  vec4f       n = {normal.x, normal.y, normal.z, 0};
  vec4f       r = {ray.d.x, ray.d.y, ray.d.z, 0};
  return c * dot(n, -r);
}

static vec4f shade_normal(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }
  const auto& material = scene.materials[isec.instance];
  const auto& shape    = scene.shapes[isec.instance];
  const auto& normal   = eval_normal(shape, isec.element, isec.uv);
  vec4f       n        = {normal.x, normal.y, normal.z, 0};
  return n * 0.5 + 0.5;
}

static vec4f shade_texcoord(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }
  const auto& shape = scene.shapes[isec.instance];
  auto  texcoordx   = fmod(eval_texcoord(shape, isec.element, isec.uv).x, 1);
  auto  texcoordy   = fmod(eval_texcoord(shape, isec.element, isec.uv).y, 1);
  vec4f res         = {texcoordx, texcoordy, 0, 1};
  return res;
}

static vec4f shade_color(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }
  const auto& object = scene.materials[isec.instance];
  vec4f       ret    = {object.color.x, object.color.y, object.color.z, 0};
  return ret;
}

// Raytrace renderer.
static vec4f shade_raytrace(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    vec3f res = eval_environment(scene, ray.d);
    return rgb_to_rgba(res);
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& material = scene.materials[instance.material];
  const auto& shape    = scene.shapes[instance.shape];

  auto normal = transform_direction(
      instance.frame, eval_normal(shape, isec.element, isec.uv));
  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  auto  texcoord = eval_texcoord(shape, isec.element, isec.uv);
  auto  mat      = eval_material(scene, instance, isec.element, isec.uv);
  auto  texture  = eval_texture(scene, material.color_tex, texcoord);
  vec4f color    = rgb_to_rgba(mat.color);

  if (rand1f(rng) < 1 - mat.opacity) {
    return shade_raytrace(
        scene, bvh, ray3f{position, ray.d}, bounce + 1, rng, params);
  }

  auto  radiance = material.emission;
  vec4f rad      = rgb_to_rgba(radiance);

  if (bounce >= params.bounces) {
    return rad;
  }

  if (!shape.points.empty()) {
    normal = -ray.d;
  } else if (!shape.lines.empty()) {
    normal = orthonormalize(-ray.d, normal);
  } else if (!shape.triangles.empty()) {
    if (dot(-ray.d, normal) < 0) {
      normal = -normal;
    }
  }

  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      rad += color * shade_raytrace(scene, bvh, ray3f{position, incoming},
                         bounce + 1, rng, params);
      return rad;
    }
    case material_type::reflective: {
      if (mat.roughness == 0) {
        auto  incoming = reflect(-ray.d, normal);
        vec3f fresnel  = fresnel_schlick(mat.color, normal, ray.d);
        vec4f fre      = rgb_to_rgba(fresnel);
        rad += fre * shade_raytrace(scene, bvh, ray3f{position, incoming},
                         bounce + 1, rng, params);

      } else {
        auto exponent = 2 / (mat.roughness * mat.roughness);
        auto halfway  = sample_hemisphere_cospower(
            exponent, normal, rand2f(rng));
        auto incoming = reflect(-ray.d, halfway);
        rad += color * shade_raytrace(scene, bvh, ray3f{position, incoming},
                           bounce + 1, rng, params);
      }
      return rad;
    }
    case material_type::glossy: {
      auto  exponent = 2 / (mat.roughness * mat.roughness);
      auto  halfway = sample_hemisphere_cospower(exponent, normal, rand2f(rng));
      vec3f vec004  = {0.04, 0.04, 0.04};
      auto  fre     = fresnel_schlick(vec004, halfway, -ray.d);
      if (rand1f(rng) < fre.x) {
        auto incoming = reflect(-ray.d, halfway);
        rad += shade_raytrace(
            scene, bvh, ray3f{position, incoming}, bounce + 1, rng, params);
      } else {
        auto incoming = sample_hemisphere_cos(normal, rand2f(rng));
        rad += color * shade_raytrace(scene, bvh, ray3f{position, incoming},
                           bounce + 1, rng, params);
      }
      return rad;
    }
    case material_type::transparent: {
      vec3f vec004 = {0.04, 0.04, 0.04};
      auto  fre    = fresnel_schlick(vec004, normal, ray.d);
      if (rand1f(rng) < fre.x) {
        auto incoming = reflect(-ray.d, normal);
        rad += shade_raytrace(
            scene, bvh, ray3f{position, incoming}, bounce + 1, rng, params);
      } else {
        auto incoming = ray.d;
        rad += color * shade_raytrace(scene, bvh, ray3f{position, incoming},
                           bounce + 1, rng, params);
      }
      return rad;
    }
  }
}

static vec4f shade_sand(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    vec3f res = eval_environment(scene, ray.d);
    return rgb_to_rgba(res);
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& material = scene.materials[instance.material];
  const auto& shape    = scene.shapes[instance.shape];

  auto normal = transform_direction(
      instance.frame, eval_normal(shape, isec.element, isec.uv));
  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  auto  texcoord = eval_texcoord(shape, isec.element, isec.uv);
  auto  mat      = eval_material(scene, instance, isec.element, isec.uv);
  auto  texture  = eval_texture(scene, material.color_tex, texcoord);
  vec4f color    = rgb_to_rgba(mat.color);

  auto  radiance = material.emission;
  vec4f rad      = rgb_to_rgba(radiance);

  if (bounce >= params.bounces) {
    return rad;
  }

  if (!shape.points.empty()) {
    normal = -ray.d;
  } else if (!shape.lines.empty()) {
    normal = orthonormalize(-ray.d, normal);
  } else if (!shape.triangles.empty()) {
    if (dot(-ray.d, normal) < 0) {
      normal = -normal;
    }
  }

  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      rad += (2 * M_PI) * color / M_PI *
             shade_sand(scene, bvh, ray3f{position, incoming}, bounce + 1, rng,
                 params) *
             dot(normal, incoming);
      return rad;
    }
    case material_type::glossy: {
      auto exponent = M_PI / pow(mat.roughness, M_PI * M_PI);
      auto halfway  = sample_hemisphere_cospower(exponent, normal, rand2f(rng));

      vec3f randvec    = rand3f(rng);
      auto  reflection = reflect(halfway, randvec);

      auto  fresnel = fresnel_schlick(mat.color * 0.8, randvec, halfway);
      vec4f fre     = rgb_to_rgba(fresnel);
      rad += fre * shade_sand(scene, bvh, ray3f{position, reflection},
                       bounce + 1, rng, params);
      return rad;
    }
  }
}

static vec4f shade_stripes(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    vec3f res = eval_environment(scene, ray.d);
    return rgb_to_rgba(res);
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  const auto& material = scene.materials[instance.material];
  auto        normal   = eval_normal(shape, isec.element, isec.uv);

  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  vec4f color = rgb_to_rgba(material.color);
  vec4f nor   = {normal.x, normal.y, normal.z, 0};
  vec4f ray4d = {ray.d.x, ray.d.y, ray.d.z, 0};

  auto ret = rgb_to_rgba(material.emission);

  if (bounce >= params.bounces) {
    return ret;
  }

  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      ret += color * shade_stripes(scene, bvh, ray3f{position, incoming},
                         bounce + 1, rng, params);
      return ret;
    }
    case material_type::reflective: {
      auto  reflected = reflect(-ray.d, normal);
      vec3f fresnel   = fresnel_schlick(material.color, normal, ray.d);
      vec4f fre       = rgb_to_rgba(fresnel);
      if (position.y >= 0 && position.y <= 0.02) {
        ret += fre * shade_stripes(scene, bvh, ray3f{position, reflected},
                         bounce + 1, rng, params);
      } else if (position.y >= 0.04 && position.y <= 0.06) {
        ret += fre * shade_stripes(scene, bvh, ray3f{position, reflected},
                         bounce + 1, rng, params);

      } else if (position.y >= 0.08 && position.y <= 0.1) {
        ret += fre * shade_stripes(scene, bvh, ray3f{position, reflected},
                         bounce + 1, rng, params);
      } else if (position.y >= 0.12 && position.y <= 0.14) {
        ret += fre * shade_stripes(scene, bvh, ray3f{position, reflected},
                         bounce + 1, rng, params);
      } else if (position.y >= 0.15 && position.y <= 0.18) {
        ret += fre * shade_stripes(scene, bvh, ray3f{position, reflected},
                         bounce + 1, rng, params);
      } else {
        ret += shade_stripes(
            scene, bvh, ray3f{position, ray.d}, bounce + 1, rng, params);
      }
      return ret;
    }
  }
}

static vec4f shade_weird(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    vec3f res = eval_environment(scene, ray.d);
    return rgb_to_rgba(res);
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  const auto& material = scene.materials[instance.material];
  auto        normal   = eval_normal(shape, isec.element, isec.uv);

  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  vec4f color = rgb_to_rgba(material.color);
  vec4f nor   = {normal.x, normal.y, normal.z, 0};
  vec4f ray4d = {ray.d.x, ray.d.y, ray.d.z, 0};

  auto ret = rgb_to_rgba(material.emission);

  if (bounce >= params.bounces) {
    return ret;
  }

  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      ret += (2 * M_PI) * color / M_PI *
             shade_weird(scene, bvh, ray3f{position, incoming}, bounce + 1, rng,
                 params) *
             dot(normal, -ray.d);
      dot(normal, -ray.d);
      return ret;
    }
    case material_type::volumetric: {
      auto  red         = 0;
      auto  green       = 0.3f;
      auto  blue        = 0.2f;
      auto  next        = reflect(ray.d, normal);
      auto  randomizer  = rand1f(rng);
      float lower_bound = 0.05;
      ret +=
          vec4f{red * rand1f(rng), green * rand1f(rng), blue * rand1f(rng), 0} *
          shade_weird(
              scene, bvh, ray3f{position, next}, bounce + 1, rng, params);

      return ret;
    }
  }
}

static vec4f shade_cyber(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    vec3f res = eval_environment(scene, ray.d);
    return rgb_to_rgba(res);
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  const auto& material = scene.materials[instance.material];
  auto        normal   = eval_normal(shape, isec.element, isec.uv);

  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  vec4f color = rgb_to_rgba(material.color);
  vec4f nor   = {normal.x, normal.y, normal.z, 0};
  vec4f ray4d = {ray.d.x, ray.d.y, ray.d.z, 0};

  auto ret = rgb_to_rgba(material.emission);

  if (bounce >= params.bounces) {
    return ret;
  }
  switch (material.type) {
    case material_type::matte: {
      auto incoming = sample_hemisphere(normal, rand2f(rng));
      ret += (2 * M_PI) * color / M_PI *
             shade_cyber(scene, bvh, ray3f{position, incoming}, bounce + 1, rng,
                 params) *
             dot(normal, incoming);
      return ret;
    }
    case material_type::volumetric: {
      auto  randomizer  = rand1f(rng);
      float lower_bound = 0.05;
      if (randomizer > lower_bound) {
        ret += vec4f{0, 0.7, 0, 0} * shade_cyber(scene, bvh,
                                         ray3f{position, ray.d}, bounce + 1,
                                         rng, params);
      } else {
        auto reflected = reflect(-ray.d, normal);
        ret += vec4f{0.7, 0, 0.7, 0} * shade_cyber(scene, bvh,
                                           ray3f{position, reflected},
                                           bounce + 1, rng, params);
      }
      return ret;
    }
  }
}

static vec4f shade_toon(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  const auto& material = scene.materials[instance.material];
  const auto& normal   = eval_normal(shape, isec.element, isec.uv);

  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  vec4f color = rgb_to_rgba(material.color);
  vec4f nor   = {normal.x, normal.y, normal.z, 0};
  vec4f ray4d = {ray.d.x, ray.d.y, ray.d.z, 0};

  auto incoming = sample_hemisphere(normal, rand2f(rng));
  auto NdotL    = dot(nor, -ray4d);

  vec4f envColor = {0.4, 0.4, 0.4, 1};
  vec4f rimColor = {1, 1, 1, 1};
  vec4f speColor = {0.9, 0.9, 0.9, 1};
  auto  light    = 0;
  if (NdotL > 0.5) {
    light = 1;
  } else {
    light = 0;
  }

  vec3f viewDir                 = normalize(ray.d);
  vec3f halfVector              = normalize(ray.d + position);
  float NdotH                   = dot(normal, halfVector);
  float specularIntensity       = pow(NdotL * light, 8 * 8);
  float specularIntensitySmooth = smoothstep(0.005, 0.01, specularIntensity);
  vec4f specular                = specularIntensitySmooth * speColor;

  float rimDot       = (1 - dot(viewDir, normalize(normal))) / 2;
  float rimIntensity = smoothstep(0.716 - 0.01, 0.716 + 0.01, rimDot * NdotL);
  vec4f rim          = rimIntensity * rimColor;

  return color * (light + envColor + specular + rimDot);
}

static vec4f shade_exp1(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  const auto& material = scene.materials[instance.material];
  const auto& normal   = eval_normal(shape, isec.element, isec.uv);

  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  vec4f color = rgb_to_rgba(material.color);
  vec4f nor   = {normal.x, normal.y, normal.z, 0};
  vec4f ray4d = {ray.d.x, ray.d.y, ray.d.z, 0};

  auto mod = dot(normal, -ray.d);

  return {(1 - color.x), (1 - color.y) * mod, (1 - color.z), color.w};
}

static vec4f shade_exp2(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  const auto& material = scene.materials[instance.material];
  const auto& normal   = eval_normal(shape, isec.element, isec.uv);

  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  vec4f color = rgb_to_rgba(material.color);
  vec4f nor   = {normal.x, normal.y, normal.z, 0};
  vec4f ray4d = {ray.d.x, ray.d.y, ray.d.z, 0};

  auto mod  = dot(normal, -ray.d);
  auto mod1 = 1 - dot(normal, -ray.d);
  auto mod2 = dot(ray.d, normal);

  return {color.x * mod, color.y * mod1, color.z * mod2};
}

static vec4f shade_exp3(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  const auto& material = scene.materials[instance.material];
  const auto& normal   = eval_normal(shape, isec.element, isec.uv);

  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  vec4f color = rgb_to_rgba(material.color);
  vec4f nor   = {normal.x, normal.y, normal.z, 0};
  vec4f ray4d = {ray.d.x, ray.d.y, ray.d.z, 0};

  auto modx = fmod(normal.x, 1);
  auto mody = fmod(position.y, 1);
  auto modz = fmod(normal.z, 1);

  auto ret = vec4f{modx, mody, modz, 0};

  if (bounce >= params.bounces) {
    return ret;
  }

  ret += color * shade_exp3(scene, bvh, ray3f{position, ray.d}, bounce + 1, rng,
                     params);
  return ret;
}

static vec4f shade_exp4(const scene_data& scene, const bvh_scene& bvh,
    const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params) {
  auto isec = intersect_bvh(bvh, scene, ray);
  if (!isec.hit) {
    return {0, 0, 0, 0};
  }

  const auto& instance = scene.instances[isec.instance];
  const auto& shape    = scene.shapes[instance.shape];
  const auto& material = scene.materials[instance.material];
  const auto& normal   = eval_normal(shape, isec.element, isec.uv);

  auto position = transform_point(
      instance.frame, eval_position(shape, isec.element, isec.uv));
  vec4f color = rgb_to_rgba(material.color);
  vec4f nor   = {normal.x, normal.y, normal.z, 0};
  vec4f ray4d = {ray.d.x, ray.d.y, ray.d.z, 0};

  /*
  auto modx = fmod(position.x, 1);
  auto mody = fmod(position.y, 1);
  auto modz = fmod(position.z, 1);

  auto smooth = dot(normal, -ray.d);

  return vec4f{modx, mody, modz, 0} * (smooth + 0.5);
  */

  auto modx = fmod(normal.x, 1);
  auto mody = fmod(normal.y, 1);
  auto modz = fmod(normal.z, 1);

  return {modx, mody, modz, 0};
}
// Trace a single ray from the camera using the given algorithm.
using raytrace_shader_func = vec4f (*)(const scene_data& scene,
    const bvh_scene& bvh, const ray3f& ray, int bounce, rng_state& rng,
    const raytrace_params& params);
static raytrace_shader_func get_shader(const raytrace_params& params) {
  switch (params.shader) {
    case raytrace_shader_type::raytrace: return shade_raytrace;  // added ghost
    case raytrace_shader_type::matte: return shade_matte;
    case raytrace_shader_type::eyelight: return shade_eyelight;
    case raytrace_shader_type::normal: return shade_normal;
    case raytrace_shader_type::texcoord: return shade_texcoord;
    case raytrace_shader_type::color: return shade_color;
    case raytrace_shader_type::toon: return shade_toon;        // added
    case raytrace_shader_type::sand: return shade_sand;        // added
    case raytrace_shader_type::exp1: return shade_exp1;        // added
    case raytrace_shader_type::exp2: return shade_exp2;        // added
    case raytrace_shader_type::exp3: return shade_exp3;        // added
    case raytrace_shader_type::exp4: return shade_exp4;        // added
    case raytrace_shader_type::cyber: return shade_cyber;      // added
    case raytrace_shader_type::weird: return shade_weird;      // added
    case raytrace_shader_type::stripes: return shade_stripes;  // added

    default: {
      throw std::runtime_error("sampler unknown");
      return nullptr;
    }
  }
}

// Build the bvh acceleration structure.
bvh_scene make_bvh(const scene_data& scene, const raytrace_params& params) {
  return make_bvh(scene, false, false, params.noparallel);
}

// Init a sequence of random number generators.
raytrace_state make_state(
    const scene_data& scene, const raytrace_params& params) {
  auto& camera = scene.cameras[params.camera];
  auto  state  = raytrace_state{};
  if (camera.aspect >= 1) {
    state.width  = params.resolution;
    state.height = (int)round(params.resolution / camera.aspect);
  } else {
    state.height = params.resolution;
    state.width  = (int)round(params.resolution * camera.aspect);
  }
  state.samples = 0;
  state.image.assign(state.width * state.height, {0, 0, 0, 0});
  state.hits.assign(state.width * state.height, 0);
  state.rngs.assign(state.width * state.height, {});
  auto rng_ = make_rng(1301081);
  for (auto& rng : state.rngs) {
    rng = make_rng(961748941ull, rand1i(rng_, 1 << 31) / 2 + 1);
  }
  return state;
}

// Progressively compute an image by calling trace_samples multiple times.
void raytrace_samples(raytrace_state& state, const scene_data& scene,
    const bvh_scene& bvh, const raytrace_params& params) {
  if (state.samples >= params.samples) return;
  auto& camera = scene.cameras[params.camera];
  auto  shader = get_shader(params);
  state.samples += 1;
  if (params.samples == 1) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u = (i + 0.5f) / state.width, v = (j + 0.5f) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else if (params.noparallel) {
    for (auto idx = 0; idx < state.width * state.height; idx++) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    }
  } else {
    parallel_for(state.width * state.height, [&](int idx) {
      auto i = idx % state.width, j = idx / state.width;
      auto u        = (i + rand1f(state.rngs[idx])) / state.width,
           v        = (j + rand1f(state.rngs[idx])) / state.height;
      auto ray      = eval_camera(camera, {u, v});
      auto radiance = shader(scene, bvh, ray, 0, state.rngs[idx], params);
      if (!isfinite(radiance)) radiance = {0, 0, 0};
      state.image[idx] += radiance;
      state.hits[idx] += 1;
    });
  }
}

// Check image type
static void check_image(
    const color_image& image, int width, int height, bool linear) {
  if (image.width != width || image.height != height)
    throw std::invalid_argument{"image should have the same size"};
  if (image.linear != linear)
    throw std::invalid_argument{
        linear ? "expected linear image" : "expected srgb image"};
}

// Get resulting render
color_image get_render(const raytrace_state& state) {
  auto image = make_image(state.width, state.height, true);
  get_render(image, state);
  return image;
}
void get_render(color_image& image, const raytrace_state& state) {
  check_image(image, state.width, state.height, true);
  auto scale = 1.0f / (float)state.samples;
  for (auto idx = 0; idx < state.width * state.height; idx++) {
    image.pixels[idx] = state.image[idx] * scale;
  }
}

}  // namespace yocto
