#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Arguments.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/Mesh.h>
#include <Magnum/Math/ConfigurationValue.h>
#include <Magnum/Math/DualComplex.h>
#include <Magnum/Platform/Sdl2Application.h>
#include <Magnum/Primitives/Square.h>
#include <Magnum/SceneGraph/Camera.h>
#include <Magnum/SceneGraph/Drawable.h>
#include <Magnum/SceneGraph/TranslationRotationScalingTransformation2D.h>
#include <Magnum/SceneGraph/Scene.h>
#include <Magnum/Trade/MeshData.h>
#include <Magnum/Shaders/VertexColor.h>

namespace Magnum {
    namespace Examples {

        typedef SceneGraph::Object<SceneGraph::TranslationRotationScalingTransformation2D> Object2D;
        typedef SceneGraph::Scene<SceneGraph::TranslationRotationScalingTransformation2D> Scene2D;

        using namespace Math::Literals;

        class FactorGraph2D : public Platform::Application {
        public:
            explicit FactorGraph2D(const Arguments &arguments);

        private:
            void drawEvent() override;
            void mouseMoveEvent(MouseMoveEvent &event) override;

            Scene2D _scene;
            Object2D *_cameraObject;
            SceneGraph::Camera2D *_camera;
            SceneGraph::DrawableGroup2D _drawables;
        };

        struct ColoredPoint2D {
            Vector2 position;
            Color3 color;
        };

        class PointCloudDrawable2D : public SceneGraph::Drawable2D {
        public:
            explicit PointCloudDrawable2D(Object2D &object, Vector3 color, SceneGraph::DrawableGroup2D &drawables,
                                          GL::Buffer &vertexBuffer) :
                    SceneGraph::Drawable2D{object, &drawables} {
                _color = color;
                _mesh.setPrimitive(MeshPrimitive::Points)
                        .setCount(vertexBuffer.size())
                        .addVertexBuffer(vertexBuffer, 0, Shaders::VertexColor2D::Position{},
                                         Shaders::VertexColor2D::Color3{});
            }

        private:
            GL::Mesh _mesh;
            Color3 _color;
            Shaders::VertexColor2D _shader;

            void draw(const Matrix3 &transformation, SceneGraph::Camera2D &) override {
                _shader.draw(_mesh);
            }
        };


        FactorGraph2D::FactorGraph2D(const Arguments &arguments) : Platform::Application{arguments, NoCreate} {
            /* Make it possible for the user to have some fun */
            Utility::Arguments args;
            args.addOption("transformation", "1 0 0 0").setHelp("transformation", "initial pyramid transformation")
                    .addSkippedPrefix("magnum", "engine-specific options")
                    .parse(arguments.argc, arguments.argv);

            {
                const Vector2 dpiScaling = this->dpiScaling({});
                Configuration conf;
                conf.setTitle("Magnum Box2D Example")
                        .setSize(conf.size(), dpiScaling);
                GLConfiguration glConf;
                glConf.setSampleCount(dpiScaling.max() < 2.0f ? 8 : 2);
                if (!tryCreate(conf, glConf))
                    create(conf, glConf.setSampleCount(0));
            }

            /* Configure camera */
            _cameraObject = new Object2D{&_scene};
            _camera = new SceneGraph::Camera2D{*_cameraObject};
            _camera->setAspectRatioPolicy(SceneGraph::AspectRatioPolicy::Extend)
                    .setProjectionMatrix(Matrix3::projection({20.0f, 20.0f}))
                    .setViewport(GL::defaultFramebuffer.viewport().size());

            /* Create a point cloud */
            const ColoredPoint2D data[]{
                    {{0.1f,  0.1f},  {1.0f, 1.0f, 1.0f}},
                    {{-0.1f, -0.1f}, {1.0f, 1.0f, 1.0f}},
                    {{0.1f,  -0.1f}, {1.0f, 1.0f, 1.0f}}
            };

            GL::Buffer buffer;
            buffer.setData(data);
            Object2D &box = _scene.addChild<Object2D>();
            new PointCloudDrawable2D{box, {1.0f, 1.0f, 1.0f}, _drawables, buffer};
            setSwapInterval(1);
        }

        void FactorGraph2D::mouseMoveEvent(MouseMoveEvent &event) {
            if ((event.buttons() & MouseMoveEvent::Button::Left)) {
                Vector2 delta = 4.0f * Vector2{event.relativePosition()} / Vector2{windowSize()};
            }
        }

        void FactorGraph2D::drawEvent() {
            GL::defaultFramebuffer.clear(GL::FramebufferClear::Color);
            _camera->draw(_drawables);
            swapBuffers();
            redraw();
        }

    }
}

MAGNUM_APPLICATION_MAIN(Magnum::Examples::FactorGraph2D)