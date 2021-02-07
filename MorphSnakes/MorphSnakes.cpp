
#include "MorphSnakes.h"

CImg<double> rgb2gray(const CImg<double>& img)
{
    return 0.2989*img.get_channel(0) + 0.587*img.get_channel(1) + 0.114*img.get_channel(2);
}

CImg<double> gborders(const CImg<double>& img, double alpha, double sigma)
{
    // Gaussian gradient magnitude
    auto gaussian_blur = img.get_blur(sigma, sigma, sigma, true, true);
    CImgList<double> grads = gaussian_blur.get_gradient("xy", 0);
    auto gaussian_gradient_magnitude = (grads[0].get_sqr() + grads[1].get_sqr()).get_sqrt();
    
    auto res = (1.0 + alpha * gaussian_gradient_magnitude).get_sqrt().get_pow(-1);
    return res;
}
//Standard ist 2 und nicht 3
CImg<unsigned char> circle_levelset_3D(int height, int width, int depth,
                                    const std::array<int, 3>& center,
                                    double radius,
                                    double scalerow=1.0)
{
    CImg<unsigned char> res(width, height, depth);
    for(int k = 0; k < depth; ++k)
    {
        for(int i = 0; i < height; ++i)
        {
            for(int j = 0; j < width; ++j)
            {            
                int diffy = (i - center[0]);
                int diffx = (j - center[1]);
                int diffz = (j - center[2]);
                res(j, i, k) = (radius*radius*radius - (diffx*diffx + diffy*diffy + diffz*diffz)) > 0;            
            }
        }   
    }
    
    
    return res;
}

CImg<unsigned char> circle_levelset(int height, int width,
                                    const std::array<int, 2>& center,
                                    double radius,
                                    double scalerow=1.0)
{
    CImg<unsigned char> res(width, height);
    for(int i = 0; i < height; ++i)
    {
        for(int j = 0; j < width; ++j)
        {
            int diffy = (i - center[0]);
            int diffx = (j - center[1]);
            res(j, i) = (radius*radius - (diffx*diffx + diffy*diffy)) > 0;
        }
    }
    
    return res;
}

// Conversion from CImg to morphsnakes NDImage
template<class T>
ms::NDImage<T, 3> cimg3ndimage(CImg<T>& img)
{
    ms::Shape<3> shape = {img.height(), img.width(), img.depth()};
    ms::Stride<3> stride = {img.width() * sizeof(T), sizeof(T)};
    
    return ms::NDImage<T, 3>(img.data(), shape, stride);
}

// Conversion from CImg to morphsnakes NDImage
template<class T>
ms::NDImage<T, 2> cimg2ndimage(CImg<T>& img)
{
    ms::Shape<2> shape = {img.height(), img.width()};
    ms::Stride<2> stride = {img.width() * sizeof(T), sizeof(T)};
    
    return ms::NDImage<T, 2>(img.data(), shape, stride);
}

void ACWE()
{
    // Load image
    //CImg<double> img = rgb2gray(CImg<double>("chilli_sin_carne.txt")) / 255.0;
    CImg<float> img;
    
    img.load_raw("chilli_sin_carne.txt");
    //img.display();
    // Initialize embedding function
    auto embedding = circle_levelset_3D(img.height(), img.width(), img.depth(), {365, 730, 1}, 10);
    //(embedding * 255).save_png(ACWEoutputPathStart.c_str());
    (embedding * 255).save_ascii("test");

    // Morphological ACWE
    ms::MorphACWE<float, 3> macwe(cimg3ndimage(embedding), cimg3ndimage(img), 3);
    for(int i = 0; i < 200; ++i){
        macwe.step();
    }
    // Save results
    //(embedding * 255).save_png(ACWEoutputPath.c_str());
    (embedding * 255).save_ascii("test_result");
}

void GAC()
{
    // Load image
    CImg<double> img = rgb2gray(CImg<double>(filepath.c_str())) / 255.0;

    // Initialize embedding function
    auto embedding = circle_levelset(img.height(), img.width(), {409, 615}, 80);
    (embedding * 255).save_png(GACoutputPathStart.c_str());
    
    // Compute borders and gradients
    auto gimg = gborders(img, 1000.0, 2.0);
    CImgList<double> grad_gimg = gimg.get_gradient("yx", 0);
    std::array<ms::NDImage<double, 2>, 2> grads = {cimg2ndimage(grad_gimg[0]), cimg2ndimage(grad_gimg[1])};
    
    // Morphological GAC
    ms::MorphGAC<double, 2> mgac(cimg2ndimage(embedding), cimg2ndimage(gimg), grads, 1, 0.4, -1);
    for(int i = 0; i < 110; ++i){
        mgac.step(i);
        
    }
    // Save results
    
    (embedding * 255).save_png(GACoutputPath.c_str());
}

void drawContourInPointCloud(std::vector<std::vector<std::tuple<float,float>>> coordinates, pcl:: PointCloud<pclPoint>::Ptr &cloud){
    
    for(int i = 0; i < coordinates.size(); i++){
        for(auto v : coordinates[i]){
            pclPoint point;
            point.x = ((std::get<0>(v) -  1280/2) / 1280) * 0.65 - 0.01;
            point.y = ((std::get<1>(v) -  720/2) / 720)* 0.65 - 0.04;
            point.z = 0.53;
            point.r , point.g , point.b = 255;

            cloud->push_back(point);
        }
        
    }
    
    pclfuncs::showCloud(cloud, "cloud with contour");
    int iVector = 0;
    int iTuple = 0;
}

std::vector<std::vector<std::tuple<float,float>>> GAC(pcl::PointCloud<pclPoint>::Ptr cloud)
{
     CImg<float> image;
    
    pcl::io::savePCDFile("/home/joshua/Dokumente/Bachelor/src/PCSegmentation/MorphSnakes/data.pcd", *cloud);
    
    auto embedding = circle_levelset(image.height(), image.width(), {(boundingBoxStartY + boundingBoxLengthY/2) , boundingBoxStartX + boundingBoxLengthX/2 }, boundingBoxLengthY / 4);

    
    (embedding * 255).save_png(GACoutputPathStart.c_str());
    
    // Compute borders and gradients
    auto gimg = gborders(image, 1000.0, 2.0);
    CImgList<double> grad_gimg = gimg.get_gradient("yx", 0);
    std::array<ms::NDImage<double, 2>, 2> grads = {cimg2ndimage(grad_gimg[0]), cimg2ndimage(grad_gimg[1])};
    
    // Morphological GAC
    ms::MorphGAC<double, 2> mgac(cimg2ndimage(embedding), cimg2ndimage(gimg), grads, 2, 0.3, 1);
    for(int i = 0; i < 110; ++i){
        mgac.step(i);        
    }

    // Save results    
    (embedding * 255).save_png(GACoutputPath.c_str());
    return morphsnakes::getContourCoordinates();
}

int main()
{
    //GAC();
    //ACWE();
    return 0;
}
