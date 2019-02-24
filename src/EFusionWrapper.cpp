
#include "EFusionWrapper.h"

EFusionWrapper::EFusionWrapper(int argc, char * argv[])
    : good(true),
      eFusion(nullptr),
      gui(nullptr),
      groundTruthOdometry(nullptr),
      logReader(nullptr),
      framesToSkip(0),
      resetButton(false),
      resizeStream(nullptr)
{
    std::string empty;

    confidence     = 10.0f;
    depth          = 3.0f;
    icpWeight      = 10.0f;
    icpErrThresh   = 5e-05;
    covThresh      = 1e-05;
    photoThresh    = 115;
    fernThresh     = 0.3095f;
    timeDelta      = 200;
    so3            = true;
    icpCountThresh = 40000;

    Resolution::getInstance(640, 480);
    Intrinsics::getInstance(528, 528, 320, 240);

    // Configuration
    Parse::get().arg(argc, argv, "-l", logFile);
    iclnuim         = Parse::get().arg(argc, argv, "-icl", empty) > -1;
    openLoop        = !groundTruthOdometry && Parse::get().arg(argc, argv, "-o", empty) > -1;
    reloc           = false;
    frameskip       = false;
    quiet           = false;
    fastOdom        = false;
    rewind          = false;
    frameToFrameRGB = false;  // do tracking on consequitive rgb frames instead of against a model (??)

    startTick = 1;
    endTick   = std::numeric_limits<unsigned short>::max();  //Funny bound, since we predict times in this format really!

    if (logFile.length()) {
        // could flip colors but im not doing it for now
        logReader = new RawLogReader(logFile, Parse::get().arg(argc, argv, "-f", empty) > -1);
    } else {
        throw std::invalid_argument("Incorrect log-file name.\n");
    }

    gui = new GUI(logFile.length() == 0, Parse::get().arg(argc, argv, "-sc", empty) > -1);

    gui->flipColors->Ref().Set(logReader->flipColors);
    gui->rgbOnly->Ref().Set(false);
    gui->pyramid->Ref().Set(true);
    gui->fastOdom->Ref().Set(fastOdom);
    gui->confidenceThreshold->Ref().Set(confidence);
    gui->depthCutoff->Ref().Set(depth);
    gui->icpWeight->Ref().Set(icpWeight);
    gui->so3->Ref().Set(so3);
    gui->frameToFrameRGB->Ref().Set(frameToFrameRGB);

    resizeStream = new Resize(Resolution::getInstance().width(),
                              Resolution::getInstance().height(),
                              Resolution::getInstance().width() / 2,
                              Resolution::getInstance().height() / 2);

    // call the elastic fusion constructor
}

EFusionWrapper::~EFusionWrapper()
{
    if (eFusion) {
        delete eFusion;
    }

    if (gui) {
        delete gui;
    }

    if (groundTruthOdometry) {
        delete groundTruthOdometry;
    }

    if (logReader) {
        delete logReader;
    }

    if (resizeStream) {
        delete resizeStream;
    }
}

void EFusionWrapper::init()
{
    while (good) {
        if (eFusion) {
            run();
        }

        if (eFusion == nullptr || resetButton) {
            resetButton = false;

            if (eFusion) {
                delete eFusion;
            }

            logReader->rewind();
            eFusion = new ElasticFusion(openLoop ? std::numeric_limits<int>::max() / 2 : timeDelta,
                                        icpCountThresh,
                                        icpErrThresh,
                                        covThresh,
                                        !openLoop,
                                        iclnuim,
                                        reloc,
                                        photoThresh,
                                        confidence,
                                        depth,
                                        icpWeight,
                                        fastOdom,
                                        fernThresh,
                                        so3,
                                        frameToFrameRGB,
                                        logReader->getFile());
        } else {
            break;
        }
    }
}

void EFusionWrapper::run()
{
    while (!pangolin::ShouldQuit() && !((!logReader->hasMore()) && quiet) && !(eFusion->getTick() == endTick && quiet)) {
        if (!gui->pause->Get() || pangolin::Pushed(*gui->step)) {
            if ((logReader->hasMore()) && eFusion->getTick() < endTick) {
                TICK("LogRead");
                logReader->getNext();
                TOCK("LogRead");

                if (eFusion->getTick() < startTick) {
                    eFusion->setTick(startTick);
                    logReader->fastForward(startTick);
                }

                float weightMultiplier = framesToSkip + 1;

                Eigen::Matrix4f * currentPose = 0;

                if (groundTruthOdometry) {
                    currentPose = new Eigen::Matrix4f;
                    currentPose->setIdentity();
                    *currentPose = groundTruthOdometry->getTransformation(logReader->timestamp);
                }

                eFusion->processFrame(logReader->rgb, logReader->depth, logReader->timestamp, currentPose, weightMultiplier);

                if (currentPose) {
                    delete currentPose;
                }
            }
        } else {
            eFusion->predict();
        }

        TICK("GUI");

        if (gui->followPose->Get()) {
            pangolin::OpenGlMatrix mv;

            Eigen::Matrix4f currPose = eFusion->getCurrPose();
            Eigen::Matrix3f currRot  = currPose.topLeftCorner(3, 3);

            Eigen::Quaternionf currQuat(currRot);
            Eigen::Vector3f    forwardVector(0, 0, 1);
            Eigen::Vector3f    upVector(0, iclnuim ? 1 : -1, 0);

            Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
            Eigen::Vector3f up      = (currQuat * upVector).normalized();

            Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));

            eye -= forward;

            Eigen::Vector3f at = eye + forward;

            Eigen::Vector3f z = (eye - at).normalized();   // Forward
            Eigen::Vector3f x = up.cross(z).normalized();  // Right
            Eigen::Vector3f y = z.cross(x);

            Eigen::Matrix4d m;
            m << x(0), x(1), x(2), -(x.dot(eye)),
                y(0), y(1), y(2), -(y.dot(eye)),
                z(0), z(1), z(2), -(z.dot(eye)),
                0, 0, 0, 1;

            memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

            gui->s_cam.SetModelViewMatrix(mv);
        }

        gui->preCall();

        std::stringstream stri;
        stri << eFusion->getModelToModel().lastICPCount;
        gui->trackInliers->Ref().Set(stri.str());

        std::stringstream stre;
        stre << (std::isnan(eFusion->getModelToModel().lastICPError) ? 0 : eFusion->getModelToModel().lastICPError);
        gui->trackRes->Ref().Set(stre.str());

        if (!gui->pause->Get()) {
            gui->resLog.Log((std::isnan(eFusion->getModelToModel().lastICPError) ? std::numeric_limits<float>::max() : eFusion->getModelToModel().lastICPError), icpErrThresh);
            gui->inLog.Log(eFusion->getModelToModel().lastICPCount, icpCountThresh);
        }

        Eigen::Matrix4f pose = eFusion->getCurrPose();

        if (gui->drawGlobalModel->Get()) {
            glFinish();
            TICK("Global");

            if (gui->drawFxaa->Get()) {
                gui->drawFXAA(gui->s_cam.GetProjectionModelViewMatrix(),
                              gui->s_cam.GetModelViewMatrix(),
                              eFusion->getGlobalModel().model(),
                              eFusion->getConfidenceThreshold(),
                              eFusion->getTick(),
                              eFusion->getTimeDelta(),
                              iclnuim);
            } else {
                eFusion->getGlobalModel().renderPointCloud(gui->s_cam.GetProjectionModelViewMatrix(),
                                                           eFusion->getConfidenceThreshold(),
                                                           gui->drawUnstable->Get(),
                                                           gui->drawNormals->Get(),
                                                           gui->drawColors->Get(),
                                                           gui->drawPoints->Get(),
                                                           gui->drawWindow->Get(),
                                                           gui->drawTimes->Get(),
                                                           eFusion->getTick(),
                                                           eFusion->getTimeDelta());
            }
            glFinish();
            TOCK("Global");
        }

        if (eFusion->getLost()) {
            glColor3f(1, 1, 0);
        } else {
            glColor3f(1, 0, 1);
        }
        gui->drawFrustum(pose);
        glColor3f(1, 1, 1);

        if (gui->drawDefGraph->Get()) {
            const std::vector<GraphNode *> & graph = eFusion->getLocalDeformation().getGraph();

            for (size_t i = 0; i < graph.size(); i++) {
                pangolin::glDrawCross(graph.at(i)->position(0),
                                      graph.at(i)->position(1),
                                      graph.at(i)->position(2),
                                      0.1);

                for (size_t j = 0; j < graph.at(i)->neighbours.size(); j++) {
                    pangolin::glDrawLine(graph.at(i)->position(0),
                                         graph.at(i)->position(1),
                                         graph.at(i)->position(2),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(0),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(1),
                                         graph.at(graph.at(i)->neighbours.at(j))->position(2));
                }
            }
        }

        if (eFusion->getFerns().lastClosest != -1) {
            glColor3f(1, 0, 0);
            gui->drawFrustum(eFusion->getFerns().frames.at(eFusion->getFerns().lastClosest)->pose);
            glColor3f(1, 1, 1);
        }

        const std::vector<PoseMatch> & poseMatches = eFusion->getPoseMatches();

        int maxDiff = 0;
        for (size_t i = 0; i < poseMatches.size(); i++) {
            if (poseMatches.at(i).secondId - poseMatches.at(i).firstId > maxDiff) {
                maxDiff = poseMatches.at(i).secondId - poseMatches.at(i).firstId;
            }
        }

        for (size_t i = 0; i < poseMatches.size(); i++) {
            if (gui->drawDeforms->Get()) {
                if (poseMatches.at(i).fern) {
                    glColor3f(1, 0, 0);
                } else {
                    glColor3f(0, 1, 0);
                }
                for (size_t j = 0; j < poseMatches.at(i).constraints.size(); j++) {
                    pangolin::glDrawLine(poseMatches.at(i).constraints.at(j).sourcePoint(0), poseMatches.at(i).constraints.at(j).sourcePoint(1), poseMatches.at(i).constraints.at(j).sourcePoint(2),
                                         poseMatches.at(i).constraints.at(j).targetPoint(0), poseMatches.at(i).constraints.at(j).targetPoint(1), poseMatches.at(i).constraints.at(j).targetPoint(2));
                }
            }
        }
        glColor3f(1, 1, 1);

        eFusion->normaliseDepth(0.3f, gui->depthCutoff->Get());

        for (std::map<std::string, GPUTexture *>::const_iterator it = eFusion->getTextures().begin(); it != eFusion->getTextures().end(); ++it) {
            if (it->second->draw) {
                gui->displayImg(it->first, it->second);
            }
        }

        eFusion->getIndexMap().renderDepth(gui->depthCutoff->Get());

        gui->displayImg("ModelImg", eFusion->getIndexMap().imageTex());
        gui->displayImg("Model", eFusion->getIndexMap().drawTex());

        std::stringstream strs;
        strs << eFusion->getGlobalModel().lastCount();

        gui->totalPoints->operator=(strs.str());

        std::stringstream strs2;
        strs2 << eFusion->getLocalDeformation().getGraph().size();

        gui->totalNodes->operator=(strs2.str());

        std::stringstream strs3;
        strs3 << eFusion->getFerns().frames.size();

        gui->totalFerns->operator=(strs3.str());

        std::stringstream strs4;
        strs4 << eFusion->getDeforms();

        gui->totalDefs->operator=(strs4.str());

        std::stringstream strs5;
        strs5 << eFusion->getTick() << "/" << logReader->getNumFrames();

        gui->logProgress->operator=(strs5.str());

        std::stringstream strs6;
        strs6 << eFusion->getFernDeforms();

        gui->totalFernDefs->operator=(strs6.str());

        gui->postCall();

        logReader->flipColors = gui->flipColors->Get();
        eFusion->setRgbOnly(gui->rgbOnly->Get());
        eFusion->setPyramid(gui->pyramid->Get());
        eFusion->setFastOdom(gui->fastOdom->Get());
        eFusion->setConfidenceThreshold(gui->confidenceThreshold->Get());
        eFusion->setDepthCutoff(gui->depthCutoff->Get());
        eFusion->setIcpWeight(gui->icpWeight->Get());
        eFusion->setSo3(gui->so3->Get());
        eFusion->setFrameToFrameRGB(gui->frameToFrameRGB->Get());

        resetButton = pangolin::Pushed(*gui->reset);

        if (gui->autoSettings) {
            static bool last = gui->autoSettings->Get();

            if (gui->autoSettings->Get() != last) {
                last = gui->autoSettings->Get();
                static_cast<LiveLogReader *>(logReader)->setAuto(last);
            }
        }

        Stopwatch::getInstance().sendAll();

        if (resetButton) {
            break;
        }

        if (pangolin::Pushed(*gui->save)) {
            eFusion->savePly();
        }

        TOCK("GUI");
    }
}
