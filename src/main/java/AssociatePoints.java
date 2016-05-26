import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.alg.descriptor.UtilFeature;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.gui.feature.AssociationPanel;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.feature.TupleDesc;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageGray;
import georegression.struct.point.Point2D_F64;
import org.ddogleg.struct.FastQueue;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

/**
 * Created by vitaliy on 27.05.16.
 */
public class AssociatePoints<T extends ImageGray, TD extends TupleDesc> {

    // algorithm used to detect and describe interest points
    DetectDescribePoint<T, TD> detDesc;
    // Associated descriptions together by minimizing an error metric
    AssociateDescription<TD> associate;

    // location of interest points
    public List<Point2D_F64> pointsA;
    public List<Point2D_F64> pointsB;

    Class<T> imageType;

    public AssociatePoints() {
         imageType = (Class<T>) GrayF32.class;
         detDesc = (DetectDescribePoint<T, TD>) FactoryDetectDescribe.
                surfStable(new ConfigFastHessian(1, 2, 300, 1, 9, 4, 4), null, null, imageType);
         ScoreAssociation scorer = FactoryAssociation.defaultScore(detDesc.getDescriptionType());
         associate = FactoryAssociation.greedy(scorer, Double.MAX_VALUE, true);

    }


    public void associate(BufferedImage imageA, BufferedImage imageB) {
        T inputA = ConvertBufferedImage.convertFromSingle(imageA, null, imageType);
        T inputB = ConvertBufferedImage.convertFromSingle(imageB, null, imageType);

        // stores the location of detected interest points
        pointsA = new ArrayList<Point2D_F64>();
        pointsB = new ArrayList<Point2D_F64>();

        // stores the description of detected interest points
        FastQueue<TD> descA = UtilFeature.createQueue(detDesc, 100);
        FastQueue<TD> descB = UtilFeature.createQueue(detDesc, 100);

        // describe each image using interest points
        describeImage(inputA, pointsA, descA);
        describeImage(inputB, pointsB, descB);

        // Associate features between the two images
        associate.setSource(descA);
        associate.setDestination(descB);
        associate.associate();

        // display the results
        AssociationPanel panel = new AssociationPanel(20);
        panel.setAssociation(pointsA, pointsB, associate.getMatches());
        panel.setImages(imageA, imageB);
        ShowImages.showWindow(panel, "Associated Features", true);
    }

    /**
     * Detects features inside the two images and computes descriptions at those points.
     */
    private void describeImage(T input, List<Point2D_F64> points, FastQueue<TD> descs) {
        detDesc.detect(input);

        for (int i = 0; i < detDesc.getNumberOfFeatures(); i++) {
            points.add(detDesc.getLocation(i).copy());
            descs.grow().setTo(detDesc.getDescription(i));
        }
    }
}