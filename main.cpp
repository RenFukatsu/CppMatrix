#include<stdio.h>
#include<stdarg.h>
#include<math.h>

class Mat
{
private:
    static int num_rows;
    static int num_lines;

public:
    double mat[num_rows][num_lines];

    Mat(int num_rows, int num_lines);
    //Mat& operator=(Mat& mat);
    set(double first, ...);



};

Mat::Mat(int num_rows, int num_lines)
{
    this->num_rows = num_rows;
    this->num_lines = num_lines;
}

Mat::set(double* p)
{
    for(int j=0; j<num_lines; j++)
    {
        for(int i=0; i<num_rows; i++)
        {
            mat[i][j] = p[i + j*num_rows];
        }
    }
}

int main()
{
    Mat a(3,4);

    a.set(
        {
            {0.0, 0.1, 0.2},
            {0.3, 0.4, 0.5},
            {0.6, 0.7, 0.8},
            {0.9, 1.0, 1.1}
        }
    );

    return 0;
}