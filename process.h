#define BLACK 0
#define WHITE 255
#define FALSE 0
#define TRUE !FALSE
#define DEFAULT_SIZE 512

struct charImg
{
    unsigned char **image;
    int width;
    int height;
};

struct charImg edge_image(int pgmfile, unsigned char *input_data, char *file_out, int width, int height, int set_sobel);
int **convolve(unsigned char **, int **, int, int, int);