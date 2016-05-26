import boofcv.io.UtilIO;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.GrayF32;

import java.awt.image.BufferedImage;

/**
 * Created by vitaliy on 27.05.16.
 */
public class Main {
    public static void main( String args[] ) {
        BufferedImage imageA,imageB;
        imageA = UtilImageIO.loadImage(UtilIO.pathExample("/home/vitaliy/Downloads/panorama_image1.jpg"));
        imageB = UtilImageIO.loadImage(UtilIO.pathExample("/home/vitaliy/Downloads/panorama_image2.jpg"));
        ImageStitching.stitch(imageA,imageB, GrayF32.class);

        InterestPoint.detect(imageA, GrayF32.class);
        InterestPoint.detect(imageB, GrayF32.class);

        AssociatePoints app = new AssociatePoints();
        app.associate(imageA, imageB);

    }
}
