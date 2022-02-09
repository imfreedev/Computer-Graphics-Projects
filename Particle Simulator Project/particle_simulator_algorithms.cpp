//
// Implementation for Yocto/Particle.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
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
//

#include "yocto_particle.h"

#include <yocto/yocto_geometry.h>
#include <yocto/yocto_sampling.h>
#include <yocto/yocto_shape.h>

#include <iostream>
#include <unordered_set>

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto {

particle_scene make_ptscene(
    const scene_data& ioscene, const particle_params& params) {  // scene
  auto ptscene = particle_scene{};
  // shapes
  static auto velocity = unordered_map<string, float>{
      {"floor", 0}, {"particles", 1}, {"cloth", 0}, {"collider", 0}};
  auto iid = 0;
  for (auto& ioinstance : ioscene.instances) {
    auto& ioshape         = ioscene.shapes[ioinstance.shape];
    auto& iomaterial_name = ioscene.material_names[ioinstance.material];
    if (iomaterial_name == "particles") {
      // CREATIVE START: reducing positions
      vector<vec3f> p;
      if (params.init_type != initializer_type::classic &&
          params.solver != particle_solver_type::simulate_gas &&
          params.solver != particle_solver_type::simulate_oscillation) {
        rng_state rng = make_rng(params.seed);
        for (auto pos : ioshape.positions) {
          if (rand1f(rng) < 0.2) {
            p.push_back(pos);
          }
        }
      } else {
        p = ioshape.positions;
      }
      add_particles(ptscene, iid++, ioshape.points, p, ioshape.radius, 1, 1);
      // CREATIVE END
    } else if (iomaterial_name == "cloth") {
      auto nverts = (int)ioshape.positions.size();
      add_cloth(ptscene, iid++, ioshape.quads, ioshape.positions,
          ioshape.normals, ioshape.radius, 0.5, 1 / 8000.0,
          {nverts - 1, nverts - (int)sqrt((float)nverts)});
    } else if (iomaterial_name == "collider") {
      add_collider(ptscene, iid++, ioshape.triangles, ioshape.quads,
          ioshape.positions, ioshape.normals, ioshape.radius);
    } else if (iomaterial_name == "floor") {
      add_collider(ptscene, iid++, ioshape.triangles, ioshape.quads,
          ioshape.positions, ioshape.normals, ioshape.radius);
    } else {
      throw std::invalid_argument{"unknown material " + iomaterial_name};
    }
  }
  // done
  return ptscene;
}

void update_ioscene(scene_data& ioscene, const particle_scene& ptscene) {
  for (auto& ptshape : ptscene.shapes) {
    auto& ioshape = ioscene.shapes[ptshape.shape];
    get_positions(ptshape, ioshape.positions);
    get_normals(ptshape, ioshape.normals);
  }
}

void flatten_scene(scene_data& ioscene) {
  for (auto& ioinstance : ioscene.instances) {
    auto& shape = ioscene.shapes[ioinstance.shape];
    for (auto& position : shape.positions)
      position = transform_point(ioinstance.frame, position);
    for (auto& normal : shape.normals)
      normal = transform_normal(ioinstance.frame, normal);
    ioinstance.frame = identity3x4f;
  }
}

// Scene creation
int add_particles(particle_scene& scene, int shape_id,
    const vector<int>& points, const vector<vec3f>& positions,
    const vector<float>& radius, float mass, float random_velocity) {
  auto& shape             = scene.shapes.emplace_back();
  shape.shape             = shape_id;
  shape.points            = points;
  shape.initial_positions = positions;
  shape.initial_normals.assign(shape.positions.size(), {0, 0, 1});
  shape.initial_radius = radius;
  shape.initial_invmass.assign(positions.size(), 1 / (mass * positions.size()));
  shape.initial_velocities.assign(positions.size(), {0, 0, 0});
  shape.emit_rngscale = random_velocity;
  return (int)scene.shapes.size() - 1;
}
int add_cloth(particle_scene& scene, int shape_id, const vector<vec4i>& quads,
    const vector<vec3f>& positions, const vector<vec3f>& normals,
    const vector<float>& radius, float mass, float coeff,
    const vector<int>& pinned) {
  auto& shape             = scene.shapes.emplace_back();
  shape.shape             = shape_id;
  shape.quads             = quads;
  shape.initial_positions = positions;
  shape.initial_normals   = normals;
  shape.initial_radius    = radius;
  shape.initial_invmass.assign(positions.size(), 1 / (mass * positions.size()));
  shape.initial_velocities.assign(positions.size(), {0, 0, 0});
  shape.initial_pinned = pinned;
  shape.spring_coeff   = coeff;
  return (int)scene.shapes.size() - 1;
}
int add_collider(particle_scene& scene, int shape_id,
    const vector<vec3i>& triangles, const vector<vec4i>& quads,
    const vector<vec3f>& positions, const vector<vec3f>& normals,
    const vector<float>& radius) {
  auto& collider     = scene.colliders.emplace_back();
  collider.shape     = shape_id;
  collider.quads     = quads;
  collider.triangles = triangles;
  collider.positions = positions;
  collider.normals   = normals;
  collider.radius    = radius;
  return (int)scene.colliders.size() - 1;
}

// Set shapes
void set_velocities(
    particle_shape& shape, const vec3f& velocity, float random_scale) {
  shape.emit_velocity = velocity;
  shape.emit_rngscale = random_scale;
}

// Get shape properties
void get_positions(const particle_shape& shape, vector<vec3f>& positions) {
  positions = shape.positions;
}
void get_normals(const particle_shape& shape, vector<vec3f>& normals) {
  normals = shape.normals;
}

}  // namespace yocto

// -----------------------------------------------------------------------------
// SIMULATION DATA AND API
// -----------------------------------------------------------------------------
namespace yocto {

// Init simulation
void init_simulation(particle_scene& scene, const particle_params& params) {
  // CREATIVE START
  reset_global_variables();
  // CREATIVE END
  auto      sid = 0;
  rng_state rng = make_rng(params.seed);
  for (auto& shape : scene.shapes) {
    shape.emit_rng = make_rng(params.seed, (sid++) * 2 + 1);
    shape.invmass  = shape.initial_invmass;

    // CREATIVE START
    if (params.solver == particle_solver_type::simulate_levitation) {
      for (int k = 0; k < shape.invmass.size(); k++) {
        shape.invmass[k] = (rand1f(rng) / 100) + 1;
      }
    }
    // CREATIVE END

    shape.normals    = shape.initial_normals;
    shape.positions  = shape.initial_positions;
    shape.radius     = shape.initial_radius;
    shape.velocities = shape.initial_velocities;

    shape.forces.clear();
    for (int i = 0; i < shape.positions.size(); i++)
      shape.forces.push_back(zero3f);

    for (auto index : shape.initial_pinned) shape.invmass[index] = 0;

    for (auto& velocity : shape.velocities) {
      velocity += sample_sphere(rand2f(shape.emit_rng)) * shape.emit_rngscale *
                  rand1f(shape.emit_rng);
    }

    shape.springs.clear();

    if (shape.spring_coeff > 0) {
      for (auto& edge : get_edges(shape.quads)) {
        shape.springs.push_back({edge.x, edge.y,
            distance(shape.positions[edge.x], shape.positions[edge.y]),
            shape.spring_coeff});
      }

      for (auto& quad : shape.quads) {
        shape.springs.push_back({quad.x, quad.z,
            distance(shape.positions[quad.x], shape.positions[quad.z]),
            shape.spring_coeff});
        shape.springs.push_back({quad.y, quad.w,
            distance(shape.positions[quad.y], shape.positions[quad.w]),
            shape.spring_coeff});
      }
    }
  }

  for (auto& collider : scene.colliders) {
    collider.bvh = {};

    if (collider.quads.size() > 0)
      collider.bvh = make_quads_bvh(
          collider.quads, collider.positions, collider.radius);
    else
      collider.bvh = make_triangles_bvh(
          collider.triangles, collider.positions, collider.radius);
  }
}

bool collide_collider(const particle_collider& collider, const vec3f& position,
    vec3f& hit_position, vec3f& hit_normal) {
  auto ray = ray3f{position, vec3f{0, 1, 0}};

  shape_intersection isec;

  if (collider.quads.size() > 0) {
    isec = intersect_quads_bvh(
        collider.bvh, collider.quads, collider.positions, ray);
  } else {
    isec = intersect_triangles_bvh(
        collider.bvh, collider.triangles, collider.positions, ray);
  }

  if (!isec.hit) return false;

  if (collider.quads.size() > 0) {
    auto quad    = collider.quads[isec.element];
    hit_position = interpolate_quad(collider.positions[quad.x],
        collider.positions[quad.y], collider.positions[quad.z],
        collider.positions[quad.w], isec.uv);
    hit_normal   = normalize(
        interpolate_quad(collider.normals[quad.x], collider.normals[quad.y],
            collider.normals[quad.z], collider.normals[quad.w], isec.uv));
  } else {
    auto quad    = collider.triangles[isec.element];
    hit_position = interpolate_triangle(collider.positions[quad.x],
        collider.positions[quad.y], collider.positions[quad.z], isec.uv);
    hit_normal   = normalize(interpolate_triangle(collider.normals[quad.x],
        collider.normals[quad.y], collider.normals[quad.z], isec.uv));
  }

  return dot(hit_normal, ray.d) > 0;
}

void simulate_massspring(particle_scene& scene, const particle_params& params) {
  for (auto& shape : scene.shapes) {
    shape.old_positions = shape.positions;
  }

  for (auto& shape : scene.shapes) {
    for (int i = 0; i < params.mssteps; i++) {
      auto ddt = params.deltat / params.mssteps;

      for (int k = 0; k < shape.positions.size(); k++) {
        if (!shape.invmass[k]) continue;
        shape.forces[k] = vec3f{0, -params.gravity, 0} / shape.invmass[k];
      }

      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];
        auto  invmass   = shape.invmass[spring.vert0] +
                       shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto delta_pos = particle1 - particle0;
        auto delta_vel = shape.velocities[spring.vert1] -
                         shape.velocities[spring.vert0];

        auto spring_dir = normalize(delta_pos);
        auto spring_len = length(delta_pos);

        auto force = spring_dir * (spring_len / spring.rest - 1.f) /
                     (spring.coeff * invmass);
        force += dot(delta_vel / spring.rest, spring_dir) * spring_dir /
                 (spring.coeff * 1000 * invmass);

        shape.forces[spring.vert0] += force;
        shape.forces[spring.vert1] -= force;
      }

      for (int k = 0; k < shape.positions.size(); k++) {
        if (!shape.invmass[k]) continue;
        shape.velocities[k] += ddt * shape.forces[k] * shape.invmass[k];
        shape.positions[k] += ddt * shape.velocities[k];
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (collide_collider(
                collider, shape.positions[k], hitpos, hit_normal)) {
          shape.positions[k] = hitpos + hit_normal * 0.005f;
          auto projection    = dot(shape.velocities[k], hit_normal);
          shape.velocities[k] =
              (shape.velocities[k] - projection * hit_normal) *
                  (1.f - params.bounce.x) -
              projection * hit_normal * (1.f - params.bounce.y);
        }
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
      if (length(shape.velocities[k]) < params.minvelocity)
        shape.velocities[k] = {0, 0, 0};
    }
  }

  for (auto& shape : scene.shapes) {
    if (shape.quads.size() > 0)
      shape.normals = quads_normals(shape.quads, shape.positions);
    else
      shape.normals = triangles_normals(shape.triangles, shape.positions);
  }
}

void simulate_pbd(particle_scene& scene, const particle_params& params) {
  for (auto& shape : scene.shapes) {
    shape.old_positions = shape.positions;
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] += vec3f{0, -params.gravity, 0} * params.deltat;
      shape.positions[k] += shape.velocities[k] * params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    shape.collisions.clear();
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (!collide_collider(collider, shape.positions[k], hitpos, hit_normal))
          continue;

        shape.collisions.push_back({k, hitpos, hit_normal});
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int i = 0; i < params.pdbsteps; i++) {
      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];

        auto invmass = shape.invmass[spring.vert0] +
                       shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;

        auto len = length(dir);
        dir /= len;

        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape.positions[spring.vert0] += shape.invmass[spring.vert0] * lambda *
                                         dir;
        shape.positions[spring.vert1] -= shape.invmass[spring.vert1] * lambda *
                                         dir;
      }

      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] = (shape.positions[k] - shape.old_positions[k]) /
                            params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
      if (length(shape.velocities[k]) < params.minvelocity)
        shape.velocities[k] = {0, 0, 0};
    }
  }

  for (auto& shape : scene.shapes) {
    if (shape.quads.size() > 0)
      shape.normals = quads_normals(shape.quads, shape.positions);
    else
      shape.normals = triangles_normals(shape.triangles, shape.positions);
  }
}

//|----------------------------------------------|
//|--------------| CREATIVE |--------------------|
//|----------------------------------------------|

// CREATIVE INITIALIZERS ALGORITHMS

vector<vec3f> temperatures = {};  // PARAMETRIC GAS SIMULATION

void init_neighbours_simulation(
    yocto::particle_scene& scene, yocto::particle_params const& params) {
  reset_global_variables();
  auto      sid = 0;
  rng_state rng = make_rng(params.seed);

  for (auto& shape : scene.shapes) {
    shape.emit_rng = make_rng(params.seed, (sid++) * 2 + 1);
    shape.invmass  = shape.initial_invmass;
    if (params.init_type == initializer_type::tornado) {
      for (int k = 0; k < shape.invmass.size(); k++) {
        shape.invmass[k] = rand1f(rng);
        if (params.solver == particle_solver_type::simulate_gas) {
          shape.invmass[k] /= 1000;
          temperatures.push_back(vec3f{0, params.temperature, 0});
        }
      }
      shape.emit_rngscale /= 5;
    }
    shape.normals      = shape.initial_normals;
    shape.positions    = shape.initial_positions;
    shape.radius       = shape.initial_radius;
    shape.velocities   = shape.initial_velocities;
    shape.bounce       = params.bouciness;
    shape.has_collided = false;
    shape.spring_coeff = 0.5;
    if (params.init_type == initializer_type::tornado) {
      shape.spring_coeff = 0;
    }

    shape.forces.clear();
    for (int i = 0; i < shape.positions.size(); i++)
      shape.forces.push_back(zero3f);

    for (int i = 0; i < shape.positions.size(); i++) {
      shape.positions[i] += rand3f(rng) / 20;
    }

    for (auto& velocity : shape.velocities) {
      velocity += sample_sphere(rand2f(shape.emit_rng)) * shape.emit_rngscale *
                  rand1f(shape.emit_rng);
    }

    float range = 0.01;  // [0.001 , 0.01]

    float rest_offset = 0;  // [0 , 0.1]

    for (int i = 0; i < shape.positions.size(); i++) {
      particle_neighbours particle;
      particle.particle = shape.positions[i];
      particle.index    = i;
      scene.groups.push_back(particle);
      for (int j = 0; j < shape.positions.size(); j++) {
        if (i != j &&
            distance(shape.positions[i], shape.positions[j]) < range) {
          scene.groups[i].neighIdxs.push_back(j);
          scene.groups[i].neighbours.push_back(shape.positions[j]);
        }
      }
    }

    shape.springs.clear();

    if (shape.spring_coeff > 0) {
      for (auto vert0 = 0; vert0 < shape.positions.size(); vert0++) {
        for (auto vert1 : scene.groups[vert0].neighIdxs) {
          particle_spring spring = {vert0, vert1,
              distance(shape.positions[vert0], shape.positions[vert1]) +
                  rest_offset,
              shape.spring_coeff};
          shape.springs.push_back(spring);
        }
      }
    }
  }

  for (auto& collider : scene.colliders) {
    collider.bvh = {};

    if (collider.quads.size() > 0)
      collider.bvh = make_quads_bvh(
          collider.quads, collider.positions, collider.radius);
    else
      collider.bvh = make_triangles_bvh(
          collider.triangles, collider.positions, collider.radius);
  }
}

void init_creative_simulation(
    particle_scene& scene, const particle_params& params) {
  reset_global_variables();
  auto sid = 0;
  for (auto& shape : scene.shapes) {
    shape.emit_rng     = make_rng(params.seed, (sid++) * 2 + 1);
    shape.invmass      = shape.initial_invmass;
    shape.normals      = shape.initial_normals;
    shape.positions    = shape.initial_positions;
    shape.radius       = shape.initial_radius;
    shape.velocities   = shape.initial_velocities;
    shape.bounce       = params.bouciness;
    shape.has_collided = false;

    shape.forces.clear();
    for (int i = 0; i < shape.positions.size(); i++)
      shape.forces.push_back(zero3f);

    // No pinned particles

    for (auto& velocity : shape.velocities) {
      velocity += sample_sphere(rand2f(shape.emit_rng)) * shape.emit_rngscale *
                  rand1f(shape.emit_rng);
    }

    for (int i = 0; i < shape.positions.size(); i++) {
      particle_neighbours particle;
      particle.particle = shape.positions[i];
      particle.index    = i;
      scene.groups.push_back(particle);
      for (int j = 0; j < shape.positions.size(); j++) {
        if (i != j && distance(shape.positions[i], shape.positions[j]) < 0.2) {
          scene.groups[i].neighIdxs.push_back(j);
          scene.groups[i].neighbours.push_back(shape.positions[j]);
        }
      }
    }

    shape.springs.clear();

    if (shape.spring_coeff > 0) {
      for (auto& edge : get_edges(shape.quads)) {
        shape.springs.push_back({edge.x, edge.y,
            distance(shape.positions[edge.x], shape.positions[edge.y]),
            shape.spring_coeff});
      }

      for (auto& quad : shape.quads) {
        shape.springs.push_back({quad.x, quad.z,
            distance(shape.positions[quad.x], shape.positions[quad.z]),
            shape.spring_coeff});
        shape.springs.push_back({quad.y, quad.w,
            distance(shape.positions[quad.y], shape.positions[quad.w]),
            shape.spring_coeff});
      }
    }
  }

  for (auto& collider : scene.colliders) {
    collider.bvh = {};

    if (collider.quads.size() > 0)
      collider.bvh = make_quads_bvh(
          collider.quads, collider.positions, collider.radius);
    else
      collider.bvh = make_triangles_bvh(
          collider.triangles, collider.positions, collider.radius);
  }
}

// ALGORITHMS

void simulate_bounce(particle_scene& scene, const particle_params& params) {
  for (auto& shape : scene.shapes) {
    shape.old_positions = shape.positions;
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] += vec3f{0, -params.gravity, 0} * params.deltat;
      shape.positions[k] += shape.velocities[k] * params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    shape.collisions.clear();
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (!collide_collider(collider, shape.positions[k], hitpos, hit_normal))
          continue;
        shape.collisions.push_back({k, hitpos, hit_normal});
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int i = 0; i < params.pdbsteps; i++) {
      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];

        auto invmass = shape.invmass[spring.vert0] +
                       shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;
        auto len = length(dir);
        dir /= len;

        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape.positions[spring.vert0] += shape.invmass[spring.vert0] * lambda *
                                         dir;
        shape.positions[spring.vert1] -= shape.invmass[spring.vert1] * lambda *
                                         dir;
      }

      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;

        // BOUNCING BEHAVIOUR
        if (length(shape.velocities[collision.vert]) > 0.1) {
          float density = 0.01f;

          particle += -projection * collision.normal * density;

          for (int index : scene.groups[collision.vert].neighIdxs) {
            shape.positions[index] += -projection * collision.normal * density;
          }
        } else {
          shape.velocities[collision.vert] = vec3f{0, shape.bounce, 0} *
                                             params.deltat;
          particle += shape.velocities[collision.vert] * params.deltat;

          for (int index : scene.groups[collision.vert].neighIdxs) {
            shape.velocities[index] += vec3f{0, shape.bounce, 0} *
                                       collision.normal * params.deltat;
            shape.positions[index] += shape.velocities[index] * params.deltat;
          }
          shape.bounce -= 50;
        }
        // BOUNCING BEHAVIOUR END
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] = (shape.positions[k] - shape.old_positions[k]) /
                            params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
      if (length(shape.velocities[k]) < params.minvelocity)
        shape.velocities[k] = {0, 0, 0};
    }
  }

  for (auto& shape : scene.shapes) {
    if (shape.quads.size() > 0)
      shape.normals = quads_normals(shape.quads, shape.positions);
    else
      shape.normals = triangles_normals(shape.triangles, shape.positions);
  }
}

// TORNADO SIMULATION

float ang     = 220;
vec3f tornado = {0, 0, 0};

void simulate_tornado(particle_scene& scene, const particle_params& params) {
  for (auto& shape : scene.shapes) {
    shape.old_positions = shape.positions;
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;

      float radang = ang * M_PI / 180;
      tornado.x    = cos(radang);
      tornado.z    = sin(radang);

      rng_state rng = make_rng(params.seed);

      shape.velocities[k] +=
          (vec3f{0, -params.gravity * shape.invmass[k] / 2, 0} +
              tornado / shape.invmass[k]) *
          params.deltat;
      shape.positions[k] += shape.velocities[k] * params.deltat;

      if (ang == 360) {
        ang = 0;
      } else {
        ang += 0.006;
      }
    }
  }

  for (auto& shape : scene.shapes) {
    shape.collisions.clear();
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (!collide_collider(collider, shape.positions[k], hitpos, hit_normal))
          continue;
        shape.collisions.push_back({k, hitpos, hit_normal});
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int i = 0; i < params.pdbsteps; i++) {
      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];

        auto invmass = shape.invmass[spring.vert0] +
                       shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;
        auto len = length(dir);
        dir /= len;

        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape.positions[spring.vert0] += shape.invmass[spring.vert0] * lambda *
                                         dir;
        shape.positions[spring.vert1] -= shape.invmass[spring.vert1] * lambda *
                                         dir;
      }

      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
    }
  }
  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] = (shape.positions[k] - shape.old_positions[k]) /
                            params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
    }
  }
}

// PARAMETRIC WIND SIMULATION

vec3f wind = {0, 0, 0};

void simulate_wind(particle_scene& scene, const particle_params& params) {
  for (auto& shape : scene.shapes) {
    shape.old_positions = shape.positions;
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;

      wind.x = params.xwind;
      wind.y = params.ywind;
      wind.z = params.zwind;

      shape.velocities[k] += (vec3f{0, -params.gravity, 0} + wind) *
                             params.deltat;
      shape.positions[k] += shape.velocities[k] * params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    shape.collisions.clear();
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (!collide_collider(collider, shape.positions[k], hitpos, hit_normal))
          continue;
        shape.collisions.push_back({k, hitpos, hit_normal});
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int i = 0; i < params.pdbsteps; i++) {
      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];

        auto invmass = shape.invmass[spring.vert0] +
                       shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;
        auto len = length(dir);
        dir /= len;

        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape.positions[spring.vert0] += shape.invmass[spring.vert0] * lambda *
                                         dir;
        shape.positions[spring.vert1] -= shape.invmass[spring.vert1] * lambda *
                                         dir;
      }

      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] = (shape.positions[k] - shape.old_positions[k]) /
                            params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
      if (length(shape.velocities[k]) < params.minvelocity)
        shape.velocities[k] = {0, 0, 0};
    }
  }

  for (auto& shape : scene.shapes) {
    if (shape.quads.size() > 0)
      shape.normals = quads_normals(shape.quads, shape.positions);
    else
      shape.normals = triangles_normals(shape.triangles, shape.positions);
  }
}

// PARAMETRIC GAS SIMULATION

vec3f pressure_noise = {1, 0, 1};
vec3f air_resistance = {0, 0.3, 0};

void simulate_gas(particle_scene& scene, const particle_params& params) {
  for (auto& shape : scene.shapes) {
    shape.old_positions = shape.positions;
  }

  rng_state rng = make_rng(params.seed);

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;

      vec3f weightforce = {0, -params.gravity * shape.invmass[k], 0};

      shape.velocities[k] += (weightforce)*params.deltat;
      shape.velocities[k] += temperatures[k] * params.deltat;
      if (shape.positions[k].y < 0.15) {
        shape.velocities[k] +=
            (pressure_noise * normalize(shape.positions[k]) * rand1f(rng)) *
            params.deltat;
      }

      if (shape.velocities[k].y < 0) {
        shape.velocities[k] += air_resistance * params.deltat;
      }

      shape.positions[k] += shape.velocities[k] * params.deltat;

      if (temperatures[k].y > 0) {
        temperatures[k].y -= 0.4;
      }
    }
  }

  for (auto& shape : scene.shapes) {
    shape.collisions.clear();
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (!collide_collider(collider, shape.positions[k], hitpos, hit_normal))
          continue;
        shape.collisions.push_back({k, hitpos, hit_normal});
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int i = 0; i < params.pdbsteps; i++) {
      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];

        auto invmass = shape.invmass[spring.vert0] +
                       shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;
        auto len = length(dir);
        dir /= len;

        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape.positions[spring.vert0] += shape.invmass[spring.vert0] * lambda *
                                         dir;
        shape.positions[spring.vert1] -= shape.invmass[spring.vert1] * lambda *
                                         dir;
      }

      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] = (shape.positions[k] - shape.old_positions[k]) /
                            params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
    }
  }
}

// PARAMETRIC OSCILLATION

float angle_vib          = 0;
vec3f vibrating_function = {1, 1, 1};
float amplitude          = 500;

void simulate_oscillation(
    particle_scene& scene, const particle_params& params) {
  for (auto& shape : scene.shapes) {
    shape.old_positions = shape.positions;
  }

  rng_state rng = make_rng(params.seed);

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      float ang_vib = angle_vib * M_PI / 180;
      if (params.axis == 0) {
        shape.velocities[k].x = 0;
        vibrating_function    = {cos(ang_vib), 0, 0};
      } else if (params.axis == 1) {
        shape.velocities[k].y = 0;
        vibrating_function    = {0, cos(ang_vib), 0};
      } else if (params.axis == 2) {
        shape.velocities[k].z = 0;
        vibrating_function    = {0, 0, cos(ang_vib)};
      }

      if (!shape.invmass[k]) continue;

      shape.velocities[k] += vibrating_function * amplitude *
                             normalize(vec3f{1, 1, 1}) * params.deltat;

      shape.positions[k] += shape.velocities[k] * params.deltat;
    }
  }

  angle_vib += 30;
  if (angle_vib >= 360) {
    angle_vib = 0;
  }
  if (amplitude > 0) {
    amplitude -= 1;
  }

  for (auto& shape : scene.shapes) {
    shape.collisions.clear();
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (!collide_collider(collider, shape.positions[k], hitpos, hit_normal))
          continue;
        shape.collisions.push_back({k, hitpos, hit_normal});
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int i = 0; i < params.pdbsteps; i++) {
      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];

        auto invmass = shape.invmass[spring.vert0] +
                       shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;
        auto len = length(dir);
        dir /= len;

        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape.positions[spring.vert0] += shape.invmass[spring.vert0] * lambda *
                                         dir;
        shape.positions[spring.vert1] -= shape.invmass[spring.vert1] * lambda *
                                         dir;
      }

      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] = (shape.positions[k] - shape.old_positions[k]) /
                            params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
    }
  }
}

// LEVITATION

vec3f expl_vec  = {1, 1, 1};
vec3f lev_vec   = {0, 1, 0};
float expl_ampl = 2;
float lev_ampl  = 2;

void simulate_levitation(particle_scene& scene, const particle_params& params) {
  for (auto& shape : scene.shapes) {
    shape.old_positions = shape.positions;
  }

  rng_state rng = make_rng(params.seed);

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;

      shape.velocities[k] += vec3f{0, -params.gravity * shape.invmass[k], 0} *
                             params.deltat;

      shape.velocities[k] += expl_vec * expl_ampl * params.deltat;
      shape.velocities[k] += lev_vec * lev_ampl * params.deltat;

      shape.positions[k] += shape.velocities[k] * params.deltat;
    }
  }

  if (expl_ampl > 0) {
    expl_ampl -= 0.2;
  }
  if (expl_ampl <= 0) {
    if (lev_ampl >= params.gravity * 1.6) {
      lev_ampl = 4.1;
    } else {
      lev_ampl += 0.1;
    }
  }

  for (auto& shape : scene.shapes) {
    shape.collisions.clear();
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      for (auto& collider : scene.colliders) {
        auto hitpos = zero3f, hit_normal = zero3f;
        if (!collide_collider(collider, shape.positions[k], hitpos, hit_normal))
          continue;
        shape.collisions.push_back({k, hitpos, hit_normal});
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int i = 0; i < params.pdbsteps; i++) {
      for (auto& spring : shape.springs) {
        auto& particle0 = shape.positions[spring.vert0];
        auto& particle1 = shape.positions[spring.vert1];

        auto invmass = shape.invmass[spring.vert0] +
                       shape.invmass[spring.vert1];

        if (!invmass) continue;

        auto dir = particle1 - particle0;
        auto len = length(dir);
        dir /= len;

        auto lambda = (1.f - spring.coeff) * (len - spring.rest) / invmass;

        shape.positions[spring.vert0] += shape.invmass[spring.vert0] * lambda *
                                         dir;
        shape.positions[spring.vert1] -= shape.invmass[spring.vert1] * lambda *
                                         dir;
      }

      for (auto& collision : shape.collisions) {
        auto& particle = shape.positions[collision.vert];
        if (!shape.invmass[collision.vert]) continue;
        auto projection = dot(particle - collision.position, collision.normal);
        if (projection >= 0) continue;
        particle += -projection * collision.normal;
      }
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] = (shape.positions[k] - shape.old_positions[k]) /
                            params.deltat;
    }
  }

  for (auto& shape : scene.shapes) {
    for (int k = 0; k < shape.positions.size(); k++) {
      if (!shape.invmass[k]) continue;
      shape.velocities[k] *= (1.f - params.dumping * params.deltat);
    }
  }
}

// Simulate one step
void simulate_frame(particle_scene& scene, const particle_params& params) {
  switch (params.solver) {
    case particle_solver_type::mass_spring:
      return simulate_massspring(scene, params);
    case particle_solver_type::position_based:
      return simulate_pbd(scene, params);
    case particle_solver_type::simulate_bounce:
      return simulate_bounce(scene, params);
    case particle_solver_type::simulate_tornado:
      return simulate_tornado(scene, params);
    case particle_solver_type::simulate_wind:
      return simulate_wind(scene, params);
    case particle_solver_type::simulate_gas: return simulate_gas(scene, params);
    case particle_solver_type::simulate_oscillation:
      return simulate_oscillation(scene, params);
    case particle_solver_type::simulate_levitation:
      return simulate_levitation(scene, params);
    default: throw std::invalid_argument("unknown solver");
  }
}

// Simulate the whole sequence
void simulate_frames(particle_scene& scene, const particle_params& params,
    progress_callback progress_cb) {
  // handle progress
  auto progress = vec2i{0, 1 + (int)params.frames};

  if (progress_cb) progress_cb("init simulation", progress.x++, progress.y);
  init_simulation(scene, params);

  for (auto idx = 0; idx < params.frames; idx++) {
    if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
    simulate_frame(scene, params);
  }

  if (progress_cb) progress_cb("simulate frames", progress.x++, progress.y);
}

void reset_global_variables() {
  temperatures.clear();
  ang                = 220;
  tornado            = {0, 0, 0};
  wind               = {0, 0, 0};
  pressure_noise     = {1, 0, 1};
  air_resistance     = {0, 0.3, 0};
  angle_vib          = 0;
  vibrating_function = {1, 1, 1};
  amplitude          = 500;
  expl_vec           = {1, 1, 1};
  lev_vec            = {0, 1, 0};
  expl_ampl          = 2;
  lev_ampl           = 2;
}

}  // namespace yocto
