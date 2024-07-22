C_FLAGS := -Wall

OUT_DIR := out

LIBS := -lSDL2 -lm -lSDL2_image

$(OUT_DIR)/boids: boids.c
	gcc $(C_FLAGS) boids.c $(LIBS) -o $(OUT_DIR)/boids