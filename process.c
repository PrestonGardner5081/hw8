/* 
      Author: Thushan Perera
      Email: thushan.perera95@gmail.com
  */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "process.h"
#include "pgmfile.h"
#include "malloc_image.h"

/* open image file from file_in and write it out to file_out */
struct charImg edge_image(int pgmfile, unsigned char *input_data, char *file_out, int width, int height, int set_sobel)
{

  unsigned char **image_in;  /* specify image array - char image */
  unsigned char **image_out; /* specify image array - char image */
  struct pgmfile pg;

  /* Robert's cross masks */
  int **mask_one;
  int **mask_two;

  /* Sobel masks */
  int **smask_one;
  int **smask_two;

  int **first;
  int **second;

  int temp;            // Holds current convoluted value
  int threshold = 200; // Darker images will require lower thresholds

  /* if correct PGM file format  then get height and width */

  pg.pgm_height = height;
  pg.pgm_width = width;
  pg.pgm_depth = 255;

  //printf("Image size: Height %d Width %d\n", width, height);

  /* allocate memory for image */

  image_in = malloc_char_image(width, height);

  /* if file in PGM format then read info into predefined structure */
  /* else read raw file */

  read_pgm_image(image_in, input_data, width, height);

  image_out = malloc_char_image(width, height);

  /** Do the extra processing here */

  /* Allocate memory for the 2 robert's cross masks */
  mask_one = malloc(2 * sizeof(int *));
  mask_two = malloc(2 * sizeof(int *));
  for (int i = 0; i < 2; i++)
  {
    mask_one[i] = malloc(2 * sizeof(int));
    mask_two[i] = malloc(2 * sizeof(int));
  }

  /* Allocate memory for the 2 sobel masks */
  smask_one = malloc(3 * sizeof(int *));
  smask_two = malloc(3 * sizeof(int *));
  for (int i = 0; i < 3; i++)
  {
    smask_one[i] = malloc(3 * sizeof(int));
    smask_two[i] = malloc(3 * sizeof(int));
  }

  /* Assign values of Robert's cross mask one */
  mask_one[0][0] = 1;
  mask_one[0][1] = 0;
  mask_one[1][0] = 0;
  mask_one[1][1] = -1;

  /* Assign values of Robert's cross mask two */
  mask_two[0][0] = 0;
  mask_two[0][1] = 1;
  mask_two[1][0] = -1;
  mask_two[1][1] = 0;

  /* Assign values of Sobel vertical mask */
  smask_one[0][0] = -1;
  smask_one[0][1] = 0;
  smask_one[0][2] = 1;
  smask_one[1][0] = -2;
  smask_one[1][1] = 0;
  smask_one[1][2] = 2;
  smask_one[2][0] = -1;
  smask_one[2][1] = 0;
  smask_one[2][2] = 1;

  /* Assign values of Sobel horizontal mask */
  smask_two[0][0] = 1;
  smask_two[0][1] = 2;
  smask_two[0][2] = 1;
  smask_two[1][0] = 0;
  smask_two[1][1] = 0;
  smask_two[1][2] = 0;
  smask_two[2][0] = -1;
  smask_two[2][1] = -2;
  smask_two[2][2] = -1;

  if (set_sobel == TRUE)
  { /* Did user select sobel or not? */
    //printf("\nWill be using Sobel edge detection...\n");

    first = convolve(image_in, smask_one, 3, width, height);
    second = convolve(image_in, smask_two, 3, width, height);
  }
  else
  {
    //printf("\nWill be using Robert's Cross edge detection...\n");

    first = convolve(image_in, mask_one, 2, width, height);
    second = convolve(image_in, mask_two, 2, width, height);
  }

  /* Get convolution result and add them to get the final pixel */
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      temp = abs(first[i][j]) + abs(second[i][j]);
      if (temp > threshold)
      {
        temp = threshold;
      }
      image_out[i][j] = (unsigned char)temp;
    }
  }

  /* Memory cleanup */
  free(mask_one);
  free(mask_two);
  free(smask_one);
  free(smask_two);
  free(first);
  free(second);

  /* write output image */

  if (pgmfile == TRUE)
  {
    write_pgm_image(image_out, file_out, &pg);
  }

  struct charImg transf_img;
  transf_img.image = image_out;
  transf_img.width = width;
  transf_img.height = height;

  return transf_img;
}

/* Given an image and a mask, it will return the convolution result.
     Note that each mask needs to be sent separately.
     Also note that this method deals with special edge cases by ignoring a 1 pixel border around the image */
int **convolve(unsigned char **image_in, int **mask, int mask_size, int width, int height)
{

  /* Copy of input image with extra height and width for working */
  int **work_image;

  /* Results of convolution will be saved to this 2D array */
  int **out_image;
  int temp;

  /* Allocate memory for workable image */
  work_image = malloc(height * sizeof(int *));
  for (int i = 0; i < height; i++)
  {
    work_image[i] = malloc(width * sizeof(int));
  }

  /* Copy contents of input image to the workable copy */
  for (int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      work_image[i][j] = (int)image_in[i][j];
    }
  }

  /* Allocate memory for output int** */
  out_image = malloc(height * sizeof(int *));
  for (int i = 0; i < height; i++)
  {
    out_image[i] = malloc(width * sizeof(int));
  }

  /* We will be ignoring the 1 pixel border around image for edge cases */
  for (int i = 1; i < height - 1; i++)
  {
    for (int j = 1; j < width - 1; j++)
    {

      /* Get pixel and the neighbours and then apply onto the mask matrix */

      if (mask_size == 3)
      { // Multiplies the 3x3 matrix with a pixel and its 8 neighbours
        temp = (work_image[i - 1][j - 1] * mask[0][0]) + (work_image[i - 1][j] * mask[0][1]) +
               (work_image[i - 1][j + 1] * mask[0][2]) + (work_image[i][j - 1] * mask[1][0]) +
               (work_image[i][j] * mask[1][1]) + (work_image[i][j + 1] * mask[1][2]) +
               (work_image[i + 1][j - 1] * mask[2][0]) + (work_image[i + 1][j] * mask[2][1]) +
               (work_image[i + 1][j + 1] * mask[2][2]);
      }
      else if (mask_size == 2)
      { // Multiplies the 2x2 matrix with pixel and its 3 neighbours
        temp = (work_image[i][j] * mask[0][0]) + (work_image[i + 1][j + 1] * mask[1][1]) +
               (work_image[i][j + 1] * mask[0][1]) + (work_image[i + 1][j] * mask[1][0]);
      }

      out_image[i][j] = temp;
    }
  }

  free(work_image);

  return out_image;
}
