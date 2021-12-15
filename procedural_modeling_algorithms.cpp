//
// Implementation for Yocto/Model
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

// -----------------------------------------------------------------------------
// INCLUDES
// -----------------------------------------------------------------------------

#include "yocto_model.h"

#include <yocto/yocto_sampling.h>

#include <iostream>

#include "ext/perlin-noise/noise1234.h"

// -----------------------------------------------------------------------------
// USING DIRECTIVES
// -----------------------------------------------------------------------------
namespace yocto {

// using directives
using std::array;
using std::string;
using std::vector;
using namespace std::string_literals;

}  // namespace yocto

// -----------------------------------------------------------------------------
// IMPLEMENTATION FOR EXAMPLE OF PROCEDURAL MODELING
// -----------------------------------------------------------------------------
namespace yocto {

float noise(const vec3f& p) { return ::noise3(p.x, p.y, p.z); }
vec2f noise2(const vec3f& p) {
  return {noise(p + vec3f{0, 0, 0}), noise(p + vec3f{3, 7, 11})};
}
vec3f noise3(const vec3f& p) {
  return {noise(p + vec3f{0, 0, 0}), noise(p + vec3f{3, 7, 11}),
      noise(p + vec3f{13, 17, 19})};
}
float fbm(const vec3f& p, int octaves) {
  auto sum    = 0.0f;
  auto weight = 1.0f;
  auto scale  = 1.0f;
  for (auto octave = 0; octave < octaves; octave++) {
    sum += weight * fabs(noise(p * scale));
    weight /= 2;
    scale *= 2;
  }
  return sum;
}
float turbulence(const vec3f& p, int octaves) {
  auto sum    = 0.0f;
  auto weight = 1.0f;
  auto scale  = 1.0f;
  for (auto octave = 0; octave < octaves; octave++) {
    sum += weight * fabs(noise(p * scale));
    weight /= 2;
    scale *= 2;
  }
  return sum;
}
float ridge(const vec3f& p, int octaves) {
  auto sum    = 0.0f;
  auto weight = 0.5f;
  auto scale  = 1.0f;
  for (auto octave = 0; octave < octaves; octave++) {
    sum += weight * (1 - fabs(noise(p * scale))) * (1 - fabs(noise(p * scale)));
    weight /= 2;
    scale *= 2;
  }
  return sum;
}

void add_polyline(shape_data& shape, const vector<vec3f>& positions,
    const vector<vec4f>& colors, float thickness = 0.0001f) {
  auto offset = (int)shape.positions.size();
  shape.positions.insert(
      shape.positions.end(), positions.begin(), positions.end());
  shape.colors.insert(shape.colors.end(), colors.begin(), colors.end());
  shape.radius.insert(shape.radius.end(), positions.size(), thickness);
  for (auto idx = 0; idx < positions.size() - 1; idx++) {
    shape.lines.push_back({offset + idx, offset + idx + 1});
  }
}

void sample_shape(vector<vec3f>& positions, vector<vec3f>& normals,
    vector<vec2f>& texcoords, const shape_data& shape, int num) {
  auto triangles  = shape.triangles;
  auto qtriangles = quads_to_triangles(shape.quads);
  triangles.insert(triangles.end(), qtriangles.begin(), qtriangles.end());
  auto cdf = sample_triangles_cdf(triangles, shape.positions);
  auto rng = make_rng(19873991);
  for (auto idx = 0; idx < num; idx++) {
    auto [elem, uv] = sample_triangles(cdf, rand1f(rng), rand2f(rng));
    auto q          = triangles[elem];
    positions.push_back(interpolate_triangle(
        shape.positions[q.x], shape.positions[q.y], shape.positions[q.z], uv));
    normals.push_back(normalize(interpolate_triangle(
        shape.normals[q.x], shape.normals[q.y], shape.normals[q.z], uv)));
    if (!texcoords.empty()) {
      texcoords.push_back(interpolate_triangle(shape.texcoords[q.x],
          shape.texcoords[q.y], shape.texcoords[q.z], uv));
    } else {
      texcoords.push_back(uv);
    }
  }
}

void make_terrain(shape_data& shape, const terrain_params& params) {
  for (int i : range(shape.positions.size())) {
    shape.positions[i] +=
        shape.normals[i] *
        ridge(shape.positions[i] * params.scale, params.octaves) *
        params.height *
        (1.f - length(shape.positions[i] - params.center) / params.size);

    float k = shape.positions[i].y / params.height;

    if (k <= 0.3) {
      shape.colors.push_back(params.bottom);
    } else if (k <= 0.6) {
      shape.colors.push_back(params.middle);
    } else {
      shape.colors.push_back(params.top);
    }
  }
  quads_normals(shape.normals, shape.quads, shape.positions);
}

void make_displacement(shape_data& shape, const displacement_params& params) {
  for (int i : range(shape.positions.size())) {
    vec3f lastPos = shape.positions[i];
    shape.positions[i] +=
        shape.normals[i] *
        turbulence(shape.positions[i] * params.scale, params.octaves) *
        params.height;

    shape.colors.push_back(interpolate_line(params.bottom, params.top,
        distance(shape.positions[i], lastPos) / params.height));
  }
  quads_normals(shape.normals, shape.quads, shape.positions);
}

void make_hair(
    shape_data& hair, const shape_data& shape, const hair_params& params) {
  shape_data shp  = shape;
  auto       size = shp.positions.size();
  sample_shape(shp.positions, shp.normals, shp.texcoords, shp, params.num);

  for (int i = size; i < shp.positions.size(); i++) {
    vector<vec3f> positions;
    vector<vec4f> colors;
    positions.push_back(shp.positions[i]);
    colors.push_back(params.bottom);
    vec3f normal = shp.normals[i];
    float t      = params.lenght / params.steps;
    for (int j : range(params.steps)) {
      auto next = positions[j] + t * normal +
                  noise3(positions[j] * params.scale) * params.strength;

      next.y -= params.gravity;
      normal = normalize(next - positions[j]);
      positions.push_back(next);
      colors.push_back(interpolate_line(params.bottom, params.top,
          distance(next, positions[0]) / params.lenght));
    }
    colors[params.steps] = params.top;
    add_polyline(hair, positions, colors);
  }
  vector<vec3f> tang = lines_tangents(hair.lines, hair.positions);

  for (int h : range(tang.size())) {
    auto  ta = tang[h];
    vec4f t  = vec4f{ta.x, ta.y, ta.z, 0.f};
    hair.tangents.push_back(t);
  }
}

void make_grass(scene_data& scene, const instance_data& object,
    const vector<instance_data>& grasses, const grass_params& params) {
  auto rng = make_rng(198767);

  for (auto grass : grasses) {
    scene.instances.push_back(grass);
  }

  auto obj = scene.shapes[object.shape];

  int size = obj.positions.size();
  sample_shape(obj.positions, obj.normals, obj.texcoords, obj, params.num);

  for (int i = size; i < obj.positions.size(); i++) {
    std::cout << std::to_string(i) + "\n";
    auto grass = grasses[rand1i(rng, grasses.size())];

    grass.frame.y = obj.normals[i];
    grass.frame.x = normalize(
        vec3f{1, 0, 0} - dot(vec3f{1, 0, 0}, grass.frame.y) * grass.frame.y);
    grass.frame.z = cross(grass.frame.x, grass.frame.y);
    grass.frame.o = obj.positions[i];

    float rand = 0.9f + rand1f(rng) * 0.1f;
    grass.frame *= scaling_frame(vec3f{rand, rand, rand});

    rand = rand1f(rng) * 2 * pif;
    grass.frame *= rotation_frame(grass.frame.y, rand);

    rand = 0.1f + rand1f(rng) * 0.1f;
    grass.frame *= rotation_frame(grass.frame.z, rand);

    scene.instances.push_back(grass);
  }
}

// CREATIVE

// PLANET GENERATOR

float planet_noise(const vec3f& p, float scale, int octaves, float persistance,
    float lacunarity, float sea_level, float peak_level) {
  float amplitude   = 1;
  float frequency   = 1;
  float noiseHeight = 0;

  for (int i = 0; i < octaves; i++) {
    vec3f newp = p * scale * frequency;

    float noiseValue = fabs(noise(newp));
    noiseHeight += noiseValue * amplitude;
    amplitude *= persistance;
    frequency *= lacunarity;
  }

  // SEA OPTION
  if (sea_level != 0 && noiseHeight <= sea_level) {
    noiseHeight = sea_level;
  }

  // PEAK OPTION
  if (peak_level != 0 && noiseHeight >= peak_level) {
    noiseHeight = peak_level;
  }

  return noiseHeight;
}

void make_planet(shape_data& shape, const planet_params& params) {
  for (int i : range(shape.positions.size())) {
    vec3f lastPos    = shape.positions[i];
    float sea_level  = 0.3;
    float peak_level = 0;

    shape.positions[i] += shape.normals[i] *
                          planet_noise(shape.positions[i], params.scale, 4,
                              params.persistance, params.lacunarity, sea_level,
                              peak_level) *
                          params.height;

    float dist = distance(shape.positions[i], lastPos) / params.height;
    if (dist <= sea_level + 0.02) {
      shape.colors.push_back(
          (interpolate_line(params.sea, params.ocean, dist)));
    } else if (dist > sea_level + 0.02 && dist < 0.45) {
      shape.colors.push_back(
          (interpolate_line(params.land, params.rock, dist)));
    } else if (dist >= 0.45 && dist < 0.7) {
      shape.colors.push_back(params.rock);
    } else {
      shape.colors.push_back(params.top);
    }
  }

  quads_normals(shape.normals, shape.quads, shape.positions);
}

// SPIDERWEB GENERATOR

void make_spiderweb(shape_data& spiderweb, const shape_data& shape,
    const spiderweb_params& params) {
  shape_data shp = shape;

  vector<vec3f> nodes;
  int           n = 1024;
  sample_shape(shp.positions, shp.normals, shp.texcoords, shp, params.num);

  rng_state rng = make_rng(172784);

  for (int i : range(n)) {
    int   index      = rand1i(rng, shp.positions.size());
    vec3f elevatePos = shp.positions[index] +
                       params.lenght * shp.normals[index] * rand1f(rng) +
                       noise3(shp.positions[index]) * params.strength;

    nodes.push_back(elevatePos);
  }

  for (int i : range(shp.positions.size())) {
    vector<vec3f> positions;
    vector<vec4f> colors;
    positions.push_back(shp.positions[i]);
    colors.push_back(params.bottom);
    vec3f normal = shp.normals[i];

    float dist = 100000;
    vec3f nextNode;
    vec3f nodeInrange[nodes.size()];

    int counter = 1;
    for (auto node : nodes) {
      if (distance(node, shp.positions[i]) < params.lenght * 2) {
        nodeInrange[counter] = node;
        counter++;
      }
    }

    if (counter != 0) {
      nextNode = nodeInrange[rand1i(rng, counter)];
      positions.push_back(nextNode);
    }

    colors.push_back(interpolate_line(params.bottom, params.top,
        distance(nextNode, positions[0]) / params.lenght));

    colors[1] = params.top;
    add_polyline(spiderweb, positions, colors);
  }

  for (auto nodei : nodes) {
    for (auto nodej : nodes) {
      if (distance(nodei, nodej) < params.lenght && nodei != nodej &&
          rand1f(rng) < 0.05) {
        vector<vec3f> connections;
        vector<vec4f> colors;

        connections.push_back(nodej);
        connections.push_back(nodei);
        float color = 255 * rand1f(rng);
        colors.push_back(srgb_to_rgb(vec4f{color, color, color, 255} / 255));
        add_polyline(spiderweb, connections, colors);
      }
    }
  }

  vector<vec3f> tang = lines_tangents(spiderweb.lines, spiderweb.positions);

  for (int h : range(tang.size())) {
    auto  ta = tang[h];
    vec4f t  = vec4f{ta.x, ta.y, ta.z, 0.f};
    spiderweb.tangents.push_back(t);
  }
}

// SPIKES GENERATOR

void make_spikes(
    shape_data& spikes, const shape_data& shape, const spikes_params& params) {
  shape_data shp = shape;

  vector<vec3f> nodes;

  auto size = shp.positions.size();
  sample_shape(shp.positions, shp.normals, shp.texcoords, shp, params.num);

  rng_state rng = make_rng(172784);

  for (int i = size; i < shp.positions.size(); i++) {
    float rand = rand1f(rng);
    if (rand < 0.001) {
      int   index      = rand1i(rng, shp.positions.size());
      vec3f elevatePos = shp.positions[index] +
                         params.lenght * shp.normals[index];
      nodes.push_back(elevatePos);
    }
  }

  for (int i : range(shp.positions.size())) {
    vector<vec3f> positions;
    vector<vec4f> colors;
    positions.push_back(shp.positions[i]);
    colors.push_back(params.bottom);
    vec3f normal = shp.normals[i];

    float dist = 100000;
    vec3f nextNode;
    vec3f nodeInrange[nodes.size()];

    for (auto node : nodes) {
      if (distance(node, shp.positions[i]) < dist) {
        dist     = distance(node, shp.positions[i]);
        nextNode = node;
      }
    }
    positions.push_back(nextNode);

    colors.push_back(interpolate_line(params.bottom, params.top,
        distance(nextNode, positions[0]) / params.lenght));

    colors[1] = params.top;
    add_polyline(spikes, positions, colors);
  }

  vector<vec3f> tang = lines_tangents(spikes.lines, spikes.positions);

  for (int h : range(tang.size())) {
    auto  ta = tang[h];
    vec4f t  = vec4f{ta.x, ta.y, ta.z, 0.f};
    spikes.tangents.push_back(t);
  }
}

// GEOMETRIC STRUCTURES GENERATOR

void make_structure(shape_data& structure, const shape_data& shape,
    const structure_params& params) {
  shape_data shp = shape;

  auto size = shp.positions.size();
  sample_shape(shp.positions, shp.normals, shp.texcoords, shp, params.num);

  rng_state rng = make_rng(172784);

  for (int i = size; i < shp.positions.size(); i++) {
    shp.positions[i] = shp.positions[i] +
                       params.lenght * normalize(shp.normals[i]);
  }

  for (int i = size; i < shp.positions.size(); i++) {
    float         dist = 100000;
    vector<vec3f> nodesInrange;
    vec3f         distnode;
    for (int h : range(20)) {
      float dist = 100000;
      for (int j = size; j < shp.positions.size(); j++) {
        if (distance(shp.positions[j], shp.positions[i]) < dist &&
            shp.positions[j] != shp.positions[i] &&
            distance(shp.positions[j], shp.positions[i]) < params.lenght) {
          bool stop = false;

          for (auto ranged : nodesInrange) {
            if (ranged == shp.positions[j]) {
              stop = true;
              break;
            }
          }

          if (stop == false) {
            dist     = distance(shp.positions[j], shp.positions[i]);
            distnode = shp.positions[j];
          }
        }
      }
      nodesInrange.push_back(distnode);
    }

    for (auto node : nodesInrange) {
      if (shp.positions[i] != node) {
        vector<vec3f> connections;
        vector<vec4f> colors;
        std::cout << std::to_string(node.x) + "\n";
        connections.push_back(shp.positions[i]);
        connections.push_back(node);
        float color = 255;
        // colors.push_back(srgb_to_rgb(vec4f{color, color, color, 255} / 255));
        colors.push_back(rgb_to_rgba(rand3f(rng)));

        auto dist = distance(node, shp.positions[i]);
        add_polyline(structure, connections, colors);
      }
    }
    std::cout << "\n\n\n";
  }

  vector<vec3f> tang = lines_tangents(structure.lines, structure.positions);

  for (int h : range(tang.size())) {
    auto  ta = tang[h];
    vec4f t  = vec4f{ta.x, ta.y, ta.z, 0.f};
    structure.tangents.push_back(t);
  }
}

// PATH GENERATOR

void make_path(
    shape_data& path, const shape_data& shape, const path_params& params) {
  shape_data shp = shape;

  auto size = shp.positions.size();
  sample_shape(shp.positions, shp.normals, shp.texcoords, shp, params.num);

  rng_state rng = make_rng(172784);

  // SELECT FIRST RANDOM

  int  firstindex = rand1i(rng, shp.positions.size() - size) + size;
  bool marked[params.num];

  marked[firstindex - size] = true;

  for (int i = size; i < shp.positions.size(); i++) {
    shp.positions[i] = shp.positions[i] + params.lenght * shp.normals[i];
  }

  for (int round : range(7)) {
    vec3f root      = shp.positions[firstindex];
    int   rootIndex = firstindex - size;
    float col       = rand1f(rng);
    vec4f randcol   = rgb_to_rgba(vec3f{col, col, col});
    randcol         = rgb_to_rgba(rand3f(rng));

    for (int connections : range(12000)) {
      // MARCATURA
      marked[rootIndex] = true;

      float minDist  = 10000;
      vec3f minPoint = root;
      int   minIndex = rootIndex;

      // TROVO IL NODO MINIMO

      for (int i = size; i < shp.positions.size(); i++) {
        if (distance(root, shp.positions[i]) < minDist && i != rootIndex &&
            marked[i - size] == false) {
          minDist  = distance(root, shp.positions[i]);
          minPoint = shp.positions[i];
          minIndex = i - size;
        }
      }

      // CONNETTO IL NODO E AGGIORNO LA RADICE CON IL NUOVO NODO

      vector<vec3f> segment;
      vector<vec4f> color;

      segment.push_back(root);
      segment.push_back(minPoint);
      color.push_back(randcol);
      color.push_back(randcol / 2);

      add_polyline(path, segment, color, 0.0005f);
      root      = minPoint;
      rootIndex = minIndex;
    }
  }
}

// FOLDING FAN GENERATOR

void make_connections(shape_data& connections, const shape_data& shape,
    const connections_params& params) {
  shape_data shp  = shape;
  auto       size = shp.positions.size();
  sample_shape(shp.positions, shp.normals, shp.texcoords, shp, params.num);

  rng_state rng = make_rng(172784);

  // SELECT FIRST RANDOM

  int marked[params.num] = {0};

  for (int i = size; i < shp.positions.size(); i++) {
    shp.positions[i] = shp.positions[i] + params.lenght * shp.normals[i];
  }

  for (int round : range(1000)) {
    int   firstindex          = rand1i(rng, shp.positions.size() - size) + size;
    vec3f root                = shp.positions[firstindex];
    marked[firstindex - size] = 1;
    float col                 = rand1f(rng);
    vec4f randcol = rgb_to_rgba(vec3f{col, col, col});  // black and grey
    // randcol       = rgb_to_rgba(rand3f(rng));           // multiple colors

    for (int conn : range(10000)) {
      vector<vec3f> rangedPoints;

      // std::cout << std::to_string(root.x) + "\n";
      for (int i = size; i < shp.positions.size(); i++) {
        if (distance(root, shp.positions[i]) <
                0.005f &&  // fare rendering 1920 a 0.005 aumentando  il numero
                           // di conn e di round
            marked[i - size] == 0) {
          rangedPoints.push_back(shp.positions[i]);
          marked[i - size] = 1;
        }
      }

      float dist = 0;
      vec3f maxPoint;
      for (auto point : rangedPoints) {
        if (distance(root, point) > dist) {
          dist     = distance(root, point);
          maxPoint = point;
        }
      }

      for (auto point : rangedPoints) {
        vector<vec3f> segment;
        vector<vec4f> color;

        segment.push_back(root);
        segment.push_back(point);
        color.push_back(randcol);
        color.push_back(randcol / 2);

        add_polyline(connections, segment, color, 0.0002f);
      }

      root = maxPoint;
    }
  }
}
// VORONOI IMPLEMENTATIONS

void make_voronoi(shape_data& shape, const voronoi_params& params) {
  rng_state rng = make_rng(218394);

  // scegli key point
  vector<key> keypoints;
  for (int i : range(shape.positions.size())) {
    float rand = rand1f(rng);
    if (rand < 0.01) {
      keypoints.push_back(shape.positions[i]);
      std::cout << std::to_string(keypoints.size()) + "\n";
    }
  }

  for (int round : range(params.rounds)) {
    // Compute voroni sets

    for (int i : range(shape.positions.size())) {
      float mindist = 10000;
      int   keyindex;
      for (int k : range(keypoints.size())) {
        if (distance(shape.positions[i], keypoints[k].position) < mindist) {
          mindist  = distance(shape.positions[i], keypoints[k].position);
          keyindex = k;
        }
      }
      keypoints[keyindex].indexpoints.push_back(i);
    }

    // Update key posiiton with average position

    for (auto k : range(keypoints.size())) {
      vec3f sumpos = {0, 0, 0};
      for (int index : keypoints[k].indexpoints) {
        sumpos += shape.positions[index];
      }
      keypoints[k].position = sumpos / keypoints[k].indexpoints.size();
    }

    if (round != params.rounds - 1) {
      for (auto k : range(keypoints.size())) {
        keypoints[k].indexpoints.clear();
      }
    }
  }

  // Compute the max distance in every set

  for (auto k : range(keypoints.size())) {
    float maxdist = 0;
    for (int index : keypoints[k].indexpoints) {
      if (distance(shape.positions[index], keypoints[k].position) > maxdist) {
        maxdist = distance(shape.positions[index], keypoints[k].position);
      }
    }
    keypoints[k].maxdistance = maxdist;
  }

  // Store the position before the modifications
  vector<vec3f> lastPositions;
  for (int i : range(shape.positions.size())) {
    lastPositions.push_back(shape.positions[i]);
  }

  // Generate procedural models

  if (params.type == "RANDOM HIGH") {
    for (auto k : keypoints) {
      float noise = rand1f(rng);
      for (int index : k.indexpoints) {
        shape.positions[index] = shape.positions[index] +
                                 params.height * noise * shape.normals[index];
      }
    }
  } else if (params.type == "HOLES") {
    for (auto k : keypoints) {
      for (int index : k.indexpoints) {
        float noise = distance(shape.positions[index], k.position) /
                      k.maxdistance;

        shape.positions[index] = shape.positions[index] +
                                 params.height * noise * shape.normals[index];
      }
    }
    quads_normals(shape.normals, shape.quads, shape.positions);
  } else if (params.type == "BALLS") {
    for (auto k : keypoints) {
      for (int index : k.indexpoints) {
        float noise = distance(shape.positions[index], k.position) /
                      k.maxdistance;

        shape.positions[index] = shape.positions[index] +
                                 params.height * (1 - noise) *
                                     shape.normals[index];
      }
    }
    quads_normals(shape.normals, shape.quads, shape.positions);
  }

  for (int i : range(shape.positions.size())) {
    shape.colors.push_back(interpolate_line(params.bottom, params.top,
        distance(shape.positions[i], lastPositions[i]) / params.height));
  }
}

}  // namespace yocto
