//
// Implementation for Yocto/Grade.
//

//
// LICENSE:
//
// Copyright (c) 2020 -- 2020 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#include "yocto_colorgrade.h"

#include <yocto/yocto_color.h>
#include <yocto/yocto_sampling.h>
#include <iostream>
#include <cmath>
#include <list>

// -----------------------------------------------------------------------------
// COLOR GRADING FUNCTIONS
// -----------------------------------------------------------------------------

struct POINT{
  int x;
  int y;
  std::list<yocto::vec2f> pixels;
};

struct CIRCLE{
  int x;
  int y;
  float radius;
};

namespace yocto {

color_image grade_image(const color_image& image, const grade_params& params) {
  
  auto img = image;
  auto rng_state = make_rng(27219);

  for(auto i : range(image.width)){ 
    for(auto j : range(image.height)){     

      auto c = xyz(get_pixel(image, i, j));
      
      if(params.exposure > 0){
        c = c*pow(2,params.exposure);
      }
      
      if(params.filmic){
        c *= 0.6;
        c = (pow(c,2)*2.51 + c*0.03) / (pow(c,2)*2.43 + c*0.59 + 0.14); 
      }

      //srgb
      c = pow(c, 1/2.2);
    
      c = clamp(c, 0, 1);
      

      //Color tint
      c = c*params.tint;

      
      //Saturation
      auto g = (c.x + c.y + c.z)/3;
      c = g + (c - g)*(params.saturation*2);
      
      
      
      //Contrast
      c = gain(c, 1 - params.contrast); 
      
      
      //Vignette
      auto vr = 1 - params.vignette;
      vec2f ij = {(float) i,(float) j};
      vec2f size = {(float) img.width,(float) img.height};
      auto r = length(ij - size/2) / length(size/2);
      c = c*(1 - smoothstep(vr, 2*vr, r));
      
      //film grain
      c = c + (rand1f(rng_state) - 0.5) * params.grain;

      img[{i,j}] = {c.x, c.y, c.z, img[{i,j}].w};
    }
  }

  //Mosaic effect
  for (auto h : range(img.width))
  {
    for (auto k : range(img.height))
    {
      if(params.mosaic != 0){
        img[{h,k}] = img[{h - h%params.mosaic, k - k%params.mosaic}];
      }        
    }
  }

  //Grid effect
  for (auto h : range(img.width))
  {
    for (auto k : range(img.height))
    {
      if(params.grid != 0){
        img[{h,k}] = (0 == h%params.grid || 0 == k%params.grid) ? 0.5*img[{h,k}] : img[{h,k}];
      }       
    }
  }

  /*
    CREATIVE
  */

  //Aritmetical Average Blur

  if(params.averageblur > 0){
    auto resimg = img;
    auto blur = params.averageblur;
    std::cout << "Average blur algorithm with "+std::to_string(blur)+" starting..\n";
    for(auto i : range(img.width)){
      for(auto j : range(img.height)){
        if ( i<blur || i>=img.width-blur || j < blur || j>=img.height-blur){
          resimg[{i,j}] = img[{i,j}];
        } else {
          float sommax = 0;
          float sommay = 0;
          float sommaz = 0;
          for (auto u = -blur; u <= blur; ++u){
            for (auto v = -blur; v <= blur; ++v){
              auto c = xyz(img[{i+u,j+v}]);
              sommax += c.x;
              sommay += c.y;
              sommaz += c.z; 
            }
          }
          int div = std::pow((blur*2)+1,2);
          resimg[{i,j}] = {sommax/div, sommay/div, sommaz/div, img[{i,j}].w};
        }
      }
    }
    std::cout << "Average blurred image done.\n";

    return resimg;

  }


  // Gaussian Blur with sigma = 1.0 and variable kernel

  if(params.gaussianblur > 0){
    std::cout << "Gaussian Blur start...\n";

    float sigma = 1.0;
    auto finalimg = img;
    

    int ord = params.gaussianblur;

    if(ord%2 == 0){
      ord++;
    }

    int ordquadro = ord*ord;
    int ordmid = ((ord-1)/2);

    float kernel[ord][ord];

    for(auto i : range(ord)){
      for(auto j : range(ord)){
        kernel[i][j] =  expf((-1*(((i-j)*(i-j))))/(2*(sigma*sigma)));
      }
    }

    for(auto i : range(img.width)){
      for(auto j : range(img.height)){
        if ( i<ordmid || i>=img.width-ordmid || j<ordmid || j>=img.height-ordmid){
          finalimg[{i,j}] = img[{i,j}];
        } else {

          int ki = 0;
          float cx = 0;
          float cy = 0;
          float cz = 0;
          for (auto u = -ordmid; u <= ordmid; ++u){
            int kj = 0;
            for (auto v = -ordmid; v <= ordmid; ++v){
              auto c = xyz(img[{i+u,j+v}]);

              cx += c.x * kernel[ki][kj];
              cy += c.y * kernel[ki][kj];
              cz += c.z * kernel[ki][kj];
              kj++;
            }
            ki++;
          }
          finalimg[{i,j}] = {(cx/ordquadro)*ordmid, (cy/ordquadro)*ordmid, (cz/ordquadro)*ordmid, img[{i,j}].w};
        }
      }
    }
    std::cout << "Gaussian blurred image done.\n";
    return finalimg;

  }  


  // Popart filter

  if(params.popart > 0){
    std::cout << "Popart algorithm start...\n";
    auto finalimg = img;
    img = resize_image(img, img.width/2, img.height/2);
    for (auto i : range(img.width)){
      for (auto j : range(img.height)){
        set_pixel(finalimg,i,j, {img[{i,j}].x+0.2,img[{i,j}].y,img[{i,j}].z});
        set_pixel(finalimg,i+img.width,j, {img[{i,j}].x,img[{i,j}].y+0.2,img[{i,j}].z});
        set_pixel(finalimg,i,j+img.height, {img[{i,j}].x,img[{i,j}].y,img[{i,j}].z+1});
        set_pixel(finalimg,i+img.width,j+img.height, {img[{i,j}].x+0.2, img[{i,j}].y+0.2, img[{i,j}].z});
      }
    }
    std::cout << "Popart image done.\n";
    return finalimg;
  }

  
  // Weighted Voronoi stippling and diagrams

  if(params.voronoi > 0){
    std::cout << "Voronoi algorithm start... (10 minutes)\n";

    auto resimg = make_image(img.width,img.height,true);
    for (auto i : range(resimg.width)){
      for (auto j : range(resimg.height)){
        resimg[{i,j}] = {1,1,1,1};
        auto c = xyz(img[{i,j}]);
        auto g = (c.x + c.y + c.z)/3;
        c = g + (c - g)*0;
        img[{i,j}] = {c.x, c.y, c.z, img[{i,j}].w};
      }
    }
 
    
    POINT vorPoints[20000] = {};
    int counter = 0;
    for (auto i : range(img.width)){
      for(auto j : range(img.height)){

        auto c = xyz(img[{i,j}]);
        auto max = c.x*100+c.y*100+c.z*100;
        auto conf = (int) rand() % (int) max;

        if (conf < 4 && max <150){
          if(counter >= 20000){
            break;
          }
          POINT p = {i, j,};
          vorPoints[counter] = p;
          counter++;

        }
      }
      if(counter >= 20000){
        break;
      }
    }
    int rounds = params.voronoi;
  
    for(auto time : range(rounds)){
      for (auto i : range(img.width)){
        for(auto j : range(img.height)){
          float min = 1000000;
          int minIndex;
          for(int k : range(counter)){

            auto xpow = std::pow((vorPoints[k].x-i),2);
            auto ypow = std::pow((vorPoints[k].y-j),2);
            auto dist = sqrt(xpow + ypow);
            if(dist < min){
              min = dist;
              minIndex = k;
            }
          }
          vorPoints[minIndex].pixels.push_back({i,j});

        
        }
      }

      for(int k : range(counter)){
        float sumx = 0;
        float sumy = 0;
        float sumw = 0;

        for(auto pixel : vorPoints[k].pixels){
          auto c = xyz(img[{pixel.x,pixel.y}]);
          float weigth = 1-(c.x + c.y + c.z)/3; 

          sumx += pixel.x * weigth; 
          sumy += pixel.y * weigth;
          sumw += weigth;          
        }
        if(sumw == 0){
          sumw = 1;
        }
        sumx /= sumw; 
        sumy /= sumw; 

        vorPoints[k].x = sumx;
        vorPoints[k].y = sumy;
      }
    }


    if(params.voronoi == 1){
      for(int k : range(counter)){
        float rnx = ((double) rand()/RAND_MAX)/2.2;
        float rny = ((double) rand()/RAND_MAX)/2.2;
        float rnz = ((double) rand()/RAND_MAX)/2.2;
        for(vec2f pixel : vorPoints[k].pixels){
          auto c = xyz(img[{pixel.x, pixel.y}]);
          c = gain(c, 1 - 0.3);
          resimg[{pixel.x,pixel.y}] = {c.x-rnx, c.y-rny, c.z-rnz, img[{pixel.x,pixel.y}].w};
        }
      }
    }

    if(params.voronoi > 1){
      for(int k : range(counter)){
        
        resimg[{vorPoints[k].x+1,vorPoints[k].y}] = {0,0,0,0};
        resimg[{vorPoints[k].x-1,vorPoints[k].y}] = {0,0,0,0};
        resimg[{vorPoints[k].x,vorPoints[k].y+1}] = {0,0,0,0};
        resimg[{vorPoints[k].x,vorPoints[k].y-1}] = {0,0,0,0};
        
        resimg[{vorPoints[k].x,vorPoints[k].y}] = {0,0,0,0};
        
        resimg[{vorPoints[k].x+1,vorPoints[k].y+1}] = {0,0,0,0};
        resimg[{vorPoints[k].x-1,vorPoints[k].y-1}] = {0,0,0,0};
        resimg[{vorPoints[k].x-1,vorPoints[k].y+1}] = {0,0,0,0};
        resimg[{vorPoints[k].x+1,vorPoints[k].y-1}] = {0,0,0,0};
        
      }
    }
    
    std::cout << "Voroni image done.\n";

    return resimg;
  }
  

  
  // Experimental

  if(params.experimental > 0){
    std::cout << "Mine experimental filter start...\n";

    auto resimg = img;
    for (auto i : range(resimg.width)){
      for (auto j : range(resimg.height)){
        resimg[{i,j}] = {1,1,1,1};
      }
    }

    for(auto i : range(img.width)){
      for(auto j : range(img.height)){
        auto c = xyz(img[{i,j}]);
        auto l = xyz(resimg[{i,j}]);
        if((c.x+c.y+c.z)/3 < 0.4 && (l.x+l.y+l.z)/3 > 0.5){
          for(auto u : range(img.width)){
            auto k = xyz(resimg[{u,j}]);
            resimg[{u,j}] = {k.x-(c.x+c.y+c.z)/9,k.y-(c.x+c.y+c.z)/9,k.z-(c.x+c.y+c.z)/9,(c.x+c.y+c.z)/3};
          }
          for(auto v : range(img.height)){
            auto k = xyz(resimg[{i,v}]);
            resimg[{i,v}] = {k.x-(c.x+c.y+c.z)/9,k.y-(c.x+c.y+c.z)/9,k.z-(c.x+c.y+c.z)/9,(c.x+c.y+c.z)/3};
          }
        }
      }
    }

    for(auto i : range(img.width)){
      for(auto j : range(img.height)){
        auto c = xyz(img[{i,j}]);
        if((c.x+c.y+c.z)/3 < 0.6){
          resimg[{i,j}] = {1-c.x,1-c.y,1-c.z, img[{i,j}].w};
        }
      }
    }

    std::cout << "Experimental filter done.\n";

    return resimg;
  }



  // Splash filter

  if(params.splash > 0){
    std::cout << "Splash filter start...\n";

    auto resimg = img;
    for (auto i : range(resimg.width)){
      for (auto j : range(resimg.height)){
        resimg[{i,j}] = {0,0,0,0};
      }
    }

    for (auto i : range(resimg.width)){
      for (auto j : range(resimg.height)){
        if(i == j && i != 0){
          resimg[{i,j}] = {0,1,0.6,0};
          for(auto k : range(500)){
            if(j+k <= img.height){
              if(k<100 || (k>200 && k<300) || (k>400 && k<500) ){
                resimg[{i,j+k}] = {0,1,0.4,0};
              }  
            } 
          }
        }
        
      }
    }

    std::list<CIRCLE> circleList;
    
    for(auto k : range(25)){
      CIRCLE circle;
      bool isgood = false;

      while(!isgood){
        circle.x = (int) rand()% (int) img.width;
        circle.y = (int) rand()% (int) img.height;
        bool done = true;
        for(auto c : circleList){
          float dist = std::sqrt((c.x-circle.x)*(c.x-circle.x) + (c.y-circle.y)*(c.y-circle.y));
          if(dist < c.radius + 150){
            isgood = false;
            done = false;
            break;
          }
          
        }
        if(done){
          isgood = true;
        }
      }
      circle.radius =  rand()%100+10;
      circleList.push_back(circle);
    }

    for(auto i : range(img.width)){
      for(auto j : range(img.height)){
        
        for(auto circle : circleList){
          float dist = std::sqrt((i-circle.x)*(i-circle.x) + (j-circle.y)*(j-circle.y));


          if(dist > circle.radius+125){
            continue;
          } else if( dist <= circle.radius) {
            float k = circle.radius/2;
            if(dist < k){
              resimg[{i,j}] = {1,0,0.6,0};
            } else {
              float prob  = (float) rand()/RAND_MAX;
              if(prob < 0.3){
                resimg[{i,j}] = {1,0,0.6,0};
              }
            }
          } else {
            float prob  = (float) rand()/RAND_MAX;
            if(prob <= 0.03){
              resimg[{i,j}] = {1,0,0.6,0};
            }
          }
        }
        
      }
    }

    for(auto i : range(img.width)){
      for(auto j : range(img.height)){
        auto c = xyz(img[{i,j}]);
        if((c.x+c.y+c.z)/3 < 0.6){
          resimg[{i,j}] = {(1-c.x)/2,(1-c.y),(1-c.z)*2, img[{i,j}].w};
        }
      }
    }

    std::cout << "Splash filter done.\n";
    return resimg;
  }
  
  
  return img;
}


}  // namespace yocto