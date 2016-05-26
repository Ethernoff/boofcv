import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.abst.feature.detect.interest.InterestPointDetector;
import boofcv.factory.feature.detect.interest.FactoryInterestPoint;
import boofcv.gui.feature.FancyInterestPointRender;
import boofcv.gui.image.ShowImages;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.image.ImageGray;
import georegression.struct.point.Point2D_F64;

import java.awt.*;
import java.awt.image.BufferedImage;

public class InterestPoint {

    public static <T extends ImageGray>
    void detect(BufferedImage image, Class<T> imageType) {
        T input = ConvertBufferedImage.convertFromSingle(image, null, imageType);

        InterestPointDetector<T> detector = FactoryInterestPoint.fastHessian(
                new ConfigFastHessian(10, 2, 100, 2, 9, 3, 4));

        detector.detect(input);

        displayResults(image, detector);
    }

    private static <T extends ImageGray>
    void displayResults(BufferedImage image, InterestPointDetector<T> detector) {
        Graphics2D g2 = image.createGraphics();
        FancyInterestPointRender render = new FancyInterestPointRender();


        for (int i = 0; i < detector.getNumberOfFeatures(); i++) {
            Point2D_F64 pt = detector.getLocation(i);

            if (detector.hasScale()) {
                int radius = (int) (detector.getRadius(i));
                render.addCircle((int) pt.x, (int) pt.y, radius);
            } else {
                render.addPoint((int) pt.x, (int) pt.y);
            }
        }
        g2.setStroke(new BasicStroke(3));

        render.draw(g2);
        UtilImageIO.saveImage(image,"Detected Features"+System.currentTimeMillis()+".png");
        ShowImages.showWindow(image, "Detected Features", true);
    }
}