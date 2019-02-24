#ifndef EFUSIONWRAPPER_H
#define EFUSIONWRAPPER_H

#include <stdexcept>
#include <ElasticFusion.h>

#include "utils/GUI/GUI.h"
#include "utils/GUI/GroundTruthOdometry.h"
#include "utils/GUI/RawLogReader.h"
#include "utils/GUI/LiveLogReader.h"

/**
 * @brief Wrap and extend Elastic Fusion.
 *
 * TODO:
 *  -
 *
 */
class EFusionWrapper : public ElasticFusion
{
private:
    ElasticFusion *       eFusion;
    GUI *                 gui;
    GroundTruthOdometry * groundTruthOdometry;
    LogReader *           logReader;

    bool iclnuim,
        good;

    std::string logFile;
    std::string poseFile;

    float confidence,
        depth,
        icpWeight,
        icpErrThresh,
        covThresh,
        photoThresh,
        fernThresh;

    int timeDelta,
        icpCountThresh,
        startTick,
        endTick,
        framesToSkip;

    bool fillIn,
        openLoop,
        reloc,
        frameskip,
        quiet,
        fastOdom,
        so3,
        rewind,
        frameToFrameRGB,
        streaming,
        resetButton;

    Resize * resizeStream;

    void run();

public:
    EFusionWrapper(int argc, char * argv[]);
    ~EFusionWrapper() override;

    void init();

    /**
     * Process an rgb/depth map pair
     * @param rgb unsigned char row major order
     * @param depth unsigned short z-depth in millimeters, invalid depths
     * are 0
     * @param timestamp nanoseconds (actually only used for the output
     * poses,
     * not
     * important otherwise)
     * @param inPose optional input SE3 pose (if provided, we don't attempt
     * to
     * perform tracking)
     * @param weightMultiplier optional full frame fusion weight
     * @param bootstrap if true, use inPose as a pose guess rather than
     * replacement
     */
    void fuseFrame(const unsigned char *   rgb,
                   const unsigned short *  depth,
                   const int64_t &         timestamp,
                   const Eigen::Matrix4f * inPose           = 0,
                   const float             weightMultiplier = 1.f,
                   const bool              bootstrap        = false);
};

#endif  // EFUSIONWRAPPER_H