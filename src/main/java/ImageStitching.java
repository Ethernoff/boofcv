import boofcv.abst.feature.associate.AssociateDescription;
import boofcv.abst.feature.associate.ScoreAssociation;
import boofcv.abst.feature.detdesc.DetectDescribePoint;
import boofcv.abst.feature.detect.interest.ConfigFastHessian;
import boofcv.alg.descriptor.UtilFeature;
import boofcv.alg.distort.ImageDistort;
import boofcv.alg.distort.PixelTransformHomography_F32;
import boofcv.alg.distort.impl.DistortSupport;
import boofcv.alg.interpolate.InterpolatePixelS;
import boofcv.core.image.border.BorderType;
import boofcv.factory.feature.associate.FactoryAssociation;
import boofcv.factory.feature.detdesc.FactoryDetectDescribe;
import boofcv.factory.geo.ConfigRansac;
import boofcv.factory.geo.FactoryMultiViewRobust;
import boofcv.factory.interpolate.FactoryInterpolation;
import boofcv.gui.image.ShowImages;
import boofcv.io.UtilIO;
import boofcv.io.image.ConvertBufferedImage;
import boofcv.io.image.UtilImageIO;
import boofcv.struct.feature.AssociatedIndex;
import boofcv.struct.feature.BrightFeature;
import boofcv.struct.feature.TupleDesc;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.ImageGray;
import boofcv.struct.image.Planar;
import georegression.struct.homography.Homography2D_F64;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point2D_I32;
import georegression.transform.homography.HomographyPointOps_F64;
import org.ddogleg.fitting.modelset.ModelMatcher;
import org.ddogleg.struct.FastQueue;

import java.awt.*;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;


public class ImageStitching {

    public static<T extends ImageGray, FD extends TupleDesc> Homography2D_F64
    computeTransform( T imageA , T imageB ,
                      DetectDescribePoint<T,FD> detDesc ,
                      AssociateDescription<FD> associate ,
                      ModelMatcher<Homography2D_F64,AssociatedPair> modelMatcher )
    {
        List<Point2D_F64> pointsA = new ArrayList<Point2D_F64>();
        FastQueue<FD> descA = UtilFeature.createQueue(detDesc,100);
        List<Point2D_F64> pointsB = new ArrayList<Point2D_F64>();
        FastQueue<FD> descB = UtilFeature.createQueue(detDesc,100);

        describeImage(imageA, detDesc, pointsA, descA);
        describeImage(imageB, detDesc, pointsB, descB);


        associate.setSource(descA);
        associate.setDestination(descB);
        associate.associate();

        FastQueue<AssociatedIndex> matches = associate.getMatches();
        List<AssociatedPair> pairs = new ArrayList<AssociatedPair>();

        for( int i = 0; i < matches.size(); i++ ) {
            AssociatedIndex match = matches.get(i);

            Point2D_F64 a = pointsA.get(match.src);
            Point2D_F64 b = pointsB.get(match.dst);

            pairs.add( new AssociatedPair(a,b,false));
        }

        if( !modelMatcher.process(pairs) )
            throw new RuntimeException("Model Matcher failed!");

        return modelMatcher.getModelParameters().copy();
    }


    private static <T extends ImageGray, FD extends TupleDesc>
    void describeImage(T image,
                       DetectDescribePoint<T,FD> detDesc,
                       List<Point2D_F64> points,
                       FastQueue<FD> listDescs) {
        detDesc.detect(image);

        listDescs.reset();
        for( int i = 0; i < detDesc.getNumberOfFeatures(); i++ ) {
            points.add( detDesc.getLocation(i).copy() );
            listDescs.grow().setTo(detDesc.getDescription(i));
        }
    }


    public static <T extends ImageGray>
    void stitch( BufferedImage imageA , BufferedImage imageB , Class<T> imageType )
    {
        T inputA = ConvertBufferedImage.convertFromSingle(imageA, null, imageType);
        T inputB = ConvertBufferedImage.convertFromSingle(imageB, null, imageType);

        DetectDescribePoint detDesc = FactoryDetectDescribe.surfStable(
                new ConfigFastHessian(1, 2, 200, 1, 9, 4, 4), null,null, imageType);
        ScoreAssociation<BrightFeature> scorer = FactoryAssociation.scoreEuclidean(BrightFeature.class,true);
        AssociateDescription<BrightFeature> associate = FactoryAssociation.greedy(scorer,2,true);

        ModelMatcher<Homography2D_F64,AssociatedPair> modelMatcher =
                FactoryMultiViewRobust.homographyRansac(null,new ConfigRansac(60,3));

        Homography2D_F64 H = computeTransform(inputA, inputB, detDesc, associate, modelMatcher);

        renderStitching(imageA,imageB,H);
    }


    public static void renderStitching( BufferedImage imageA, BufferedImage imageB ,
                                        Homography2D_F64 fromAtoB )
    {
        double scale = 0.5;

        Planar<GrayF32> colorA =
                ConvertBufferedImage.convertFromMulti(imageA, null, true, GrayF32.class);
        Planar<GrayF32> colorB =
                ConvertBufferedImage.convertFromMulti(imageB, null,true, GrayF32.class);

        Planar<GrayF32> work = colorA.createSameShape();

        Homography2D_F64 fromAToWork = new Homography2D_F64(scale,0,colorA.width/4,0,scale,colorA.height/4,0,0,1);
        Homography2D_F64 fromWorkToA = fromAToWork.invert(null);

        PixelTransformHomography_F32 model = new PixelTransformHomography_F32();
        InterpolatePixelS<GrayF32> interp = FactoryInterpolation.bilinearPixelS(GrayF32.class, BorderType.ZERO);
        ImageDistort<Planar<GrayF32>,Planar<GrayF32>> distort =
                DistortSupport.createDistortPL(GrayF32.class, model, interp, false);
        distort.setRenderAll(false);

        model.set(fromWorkToA);
        distort.apply(colorA,work);

        Homography2D_F64 fromWorkToB = fromWorkToA.concat(fromAtoB,null);
        model.set(fromWorkToB);
        distort.apply(colorB,work);

        BufferedImage output = new BufferedImage(work.width,work.height,imageA.getType());
        ConvertBufferedImage.convertTo(work,output,true);

        Graphics2D g2 = output.createGraphics();

        Homography2D_F64 fromBtoWork = fromWorkToB.invert(null);
        Point2D_I32 corners[] = new Point2D_I32[4];
        corners[0] = renderPoint(0,0,fromBtoWork);
        corners[1] = renderPoint(colorB.width,0,fromBtoWork);
        corners[2] = renderPoint(colorB.width,colorB.height,fromBtoWork);
        corners[3] = renderPoint(0,colorB.height,fromBtoWork);

        g2.setColor(Color.ORANGE);
        g2.setStroke(new BasicStroke(4));
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.drawLine(corners[0].x,corners[0].y,corners[1].x,corners[1].y);
        g2.drawLine(corners[1].x,corners[1].y,corners[2].x,corners[2].y);
        g2.drawLine(corners[2].x,corners[2].y,corners[3].x,corners[3].y);
        g2.drawLine(corners[3].x,corners[3].y,corners[0].x,corners[0].y);

        UtilImageIO.saveImage(output,"Stitched Images.png");
        ShowImages.showWindow(output,"Stitched Images", true);
    }

    private static Point2D_I32 renderPoint( int x0 , int y0 , Homography2D_F64 fromBtoWork )
    {
        Point2D_F64 result = new Point2D_F64();
        HomographyPointOps_F64.transform(fromBtoWork, new Point2D_F64(x0, y0), result);
        return new Point2D_I32((int)result.x,(int)result.y);
    }


}
