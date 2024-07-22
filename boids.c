#include <stdbool.h>
#include <stdio.h>
#include "SDL2/SDL.h"
#include "SDL2/SDL_image.h"
#include <math.h>
#include <stdlib.h>
#include <time.h>

#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 720
#define SCREEN_MARGIN (SCREEN_HEIGHT + SCREEN_WIDTH) * 0.1
#define MAX_BOIDS 800

#define VISIBLE_RANGE 40
#define PROTECTED_RANGE 8
#define CENTERING_FACTOR 0.0005
#define TURN_FACTOR 1
#define AVOID_FACTOR 0.1
#define MATCHING_FACTOR 0.005

typedef struct boid {
    double angle;
    SDL_FPoint pos;
    SDL_FPoint vel;
    int width;
    int height;
} boid_t;

boid_t boids[MAX_BOIDS];
int boidsCount = 0;

float boidDistance(boid_t a, boid_t b) {
    // returns the distance between two boids
    return sqrt(pow(b.pos.x - a.pos.x, 2) + pow(b.pos.y - a.pos.y, 2));
}

double rightAngleCalc(int x1, int y1, int x2, int y2) {
    // returns 
    // distance from mouse pos and triangle pos is the hypotinuse
    float hyp = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));

    // int cx = x2;
    int cy = y1;

    // float adjacent = sqrt(pow(x1 - cx, 2));

    float opposite = sqrt(pow(y2 - cy, 2));

    return asin(opposite / hyp) * 57.2958;
    double radians = asin(opposite / hyp);
    return radians * (180.0 / M_PI);
}

void drawBoids(SDL_Renderer* renderer, SDL_Texture* boidTexture) {
    for (int i = 0; i < boidsCount; i++) {
        boid_t* current = &boids[i];
        SDL_Rect boidDrawLocation = {current->pos.x, current->pos.y, current->width, current->height};
        SDL_RenderCopyEx(
            renderer,
            boidTexture,
            NULL,
            &boidDrawLocation,
            current->angle,
            NULL,
            SDL_FLIP_NONE
        );
    }
}

double dotProd(SDL_FPoint a, SDL_FPoint b) {
    return a.x * b.x + a.y * b.y;
}

double mag(SDL_FPoint vector) {
    return sqrt(pow(vector.x, 2) + pow(vector.y, 2));
}

double directionAngle(SDL_FPoint vector) {
    SDL_FPoint i = {0, 1};
    double vectorMags = mag(vector) * mag(i);
    if (vectorMags == 0)
        return 0.0;
    double radians = acos(dotProd(vector, i) / (mag(vector) * mag(i)));
    double angle = radians * (180.0 / M_PI);
    int crossProd = (vector.x * i.y) - (vector.y * i.x);

    if (crossProd < 0)
        angle = 360.0 - angle;

    return angle;
}

void update(float dt) {
    for (int i = 0; i < boidsCount; i++) {
        boid_t* current = &boids[i];
        float xVelAgv = 0; float yVelAgv = 0;
        float xPosAvg = 0; float yPosAvg = 0;
        float closeDx = 0; float closeDy = 0;
        int visibleCount = 0;

        for (int j = 0; j < boidsCount; j++) {
            boid_t other = boids[j];
            float distance = boidDistance(*current, other);
            if (distance < PROTECTED_RANGE) {
                closeDx += current->pos.x - other.pos.x;
                closeDy += current->pos.y - other.pos.y;
            }

            if (distance < VISIBLE_RANGE) {
                visibleCount++;
                // avg vel
                xVelAgv += other.vel.x;
                yVelAgv += other.vel.y;
                // avg pos
                xPosAvg += other.pos.x;
                yPosAvg += other.pos.y;
            }
        }
        if (visibleCount > 0) {
            xVelAgv /= visibleCount;
            yVelAgv /= visibleCount;
            xPosAvg /= visibleCount;
            yPosAvg /= visibleCount;

            // alignment
            current->vel.x += (xVelAgv - current->vel.x) * MATCHING_FACTOR;
            current->vel.y += (yVelAgv - current->vel.y) * MATCHING_FACTOR;
            // cohesion
            current->vel.x += (xPosAvg - current->pos.x) * CENTERING_FACTOR;
            current->vel.y += (yPosAvg - current->pos.y) * CENTERING_FACTOR;
        }
        // seperation
        current->vel.x += (closeDx * AVOID_FACTOR);
        current->vel.y += (closeDy * AVOID_FACTOR);
        // update direction based on screen margins
        // left margin
        if (current->pos.x < SCREEN_MARGIN) {
            current->vel.x += TURN_FACTOR;
        }
        // right margin
        if (current->pos.x + current->width > SCREEN_WIDTH - SCREEN_MARGIN) {
            current->vel.x -= TURN_FACTOR;
        }
        // bottom margin
        if (current->pos.y + current->height > SCREEN_HEIGHT - SCREEN_MARGIN) {
            current->vel.y -= TURN_FACTOR;
        }
        // top margin
        if (current->pos.y < SCREEN_MARGIN) {
            current->vel.y += TURN_FACTOR;
        }
        // update angle
        current->vel.y = -current->vel.y; // to move from sdl coordinates to euclidean coordinates
        current->angle = directionAngle(current->vel);
        current->vel.y = -current->vel.y;
        // update pos
        current->pos.x += current->vel.x * dt;
        current->pos.y += current->vel.y * dt;
    }
}

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        SDL_Log("could not init sdl video");
        exit(1);
    }
    SDL_Window* window = SDL_CreateWindow(
        "idk",
        SDL_WINDOWPOS_UNDEFINED,
        SDL_WINDOWPOS_UNDEFINED,
        SCREEN_WIDTH,
        SCREEN_HEIGHT,
        0
    );
    if (window == NULL) {
        SDL_Log("could not create sdl window");
        exit(1);
    }
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    if (renderer == NULL) {
        SDL_Log("could not create sdl renderer");
        exit(1);
    }
    SDL_Texture* boidTexture = IMG_LoadTexture(renderer, "res/white-triangle.png");
    if (boidTexture == NULL) {
        SDL_Log("could not open boid texture");
        exit(1);
    }
    srand(time(NULL));
    Uint32 lastFrameTime = SDL_GetTicks();
    SDL_Event e; bool quit = false;
    bool leftMouseButtonDown = false;
    while (!quit) {
        while (SDL_PollEvent(&e) != 0) {
            if (e.type == SDL_QUIT) {
                quit = true;
            } else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                leftMouseButtonDown = true;
            } else if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) {
                leftMouseButtonDown = false;
            }

            if (leftMouseButtonDown) {
                if (boidsCount >= MAX_BOIDS)
                    break;
                SDL_FPoint pos = {e.motion.x, e.motion.y};
                int velX = (rand() % (-150 - 150 + 1)) + 150;
                int velY = (rand() % (-150 - 150 + 1)) + 150;
                SDL_FPoint vel = {velX, velY};
                
                boids[boidsCount].angle = 270;
                boids[boidsCount].pos = pos;
                boids[boidsCount].vel = vel;
                boids[boidsCount].width = 5;
                boids[boidsCount].height = 5;
                boidsCount++;
            }
        }
        // updating
        Uint32 current = SDL_GetTicks();
        float dt = (current - lastFrameTime) / 1000.0f;
        update(dt);
        lastFrameTime = current;
        // drawing
        SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xff);
        SDL_RenderClear(renderer);

        drawBoids(renderer, boidTexture);
        // draw margins
        SDL_SetRenderDrawColor(renderer, 0xff, 0xff, 0xff, 0xff);

        SDL_RenderPresent(renderer);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}