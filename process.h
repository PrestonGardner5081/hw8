#define BLACK 0
#define WHITE 255
#define FALSE 0
#define TRUE !FALSE
#define DEFAULT_SIZE 512

void edge_image(int, char *, char *, int, int, int);
int **convolve(unsigned char **, int **, int, int, int);