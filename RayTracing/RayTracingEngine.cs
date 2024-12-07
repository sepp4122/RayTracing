using System;
using System.Timers;
using System.Windows.Media;
using System.Windows.Input;
using System.Windows;
using System.Diagnostics;
using System.Windows.Controls;
using System.ComponentModel;
using System.Windows.Media.Imaging;
using System.Numerics;
using System.Text;
using Microsoft.VisualBasic;
using System.Printing;
using System.Windows.Documents;
using System.Runtime.CompilerServices;

namespace RayTracing
{
    internal class RayTracingEngine
    {
        private MainWindow mainWindow;
        private Camera camera;
        private World world;
        private System.Windows.Point previousMousePosition;
        private volatile bool isRendering = false;
        private volatile bool isRunning = true;
        private int frameRateLimiter => 1000 / Constants.fps;
        private Thread currentThread;
        public RayTracingEngine(MainWindow mainWindow)
        {
            this.mainWindow = mainWindow;
            previousMousePosition = new System.Windows.Point(mainWindow.Height / 2, mainWindow.Height / 2);
            this.mainWindow.Closed += OnWindowClosed;
            camera = new Camera(new Vector3f(0, 10, 10), (new Vector3f(0, -1, -1)).Normalize());
            world = new World();
            currentThread = new Thread(RenderLoop) { IsBackground = true };
            currentThread.Start();
        }
        private void OnWindowClosed(object sender, EventArgs e)
        {
            isRunning = false;
        }
        private void RenderLoop()
        {
            Stopwatch stopwatch = new Stopwatch();
            while (isRunning)
            {
                stopwatch.Restart();
                if (!isRendering)
                {
                    isRendering = true;
                    Constants.console.ExecuteStack(world);
                    Render();
                    isRendering = false;
                }
                Constants.console.WriteLine(stopwatch.ElapsedMilliseconds + " ms");
                int elapsed = (int)stopwatch.ElapsedMilliseconds;
                int delay = Math.Max(0, frameRateLimiter - elapsed);
                Thread.Sleep(delay);
            }
        }
        private void Render()
        {
            camera.Draw(world, mainWindow);
        }
        public void HandleKeyPress(object sender, KeyEventArgs e)
        {
            switch (e.Key)
            {
                case Key.W:
                    camera.MoveBy(new Vector3f(0, 0, -Constants.movementStepSize));
                    break;
                case Key.A:
                    camera.MoveBy(new Vector3f(-Constants.movementStepSize, 0, 0));
                    break;
                case Key.S:
                    camera.MoveBy(new Vector3f(0, 0, Constants.movementStepSize));
                    break;
                case Key.D:
                    camera.MoveBy(new Vector3f(Constants.movementStepSize, 0, 0));
                    break;
                case Key.Space:
                    camera.MoveBy(new Vector3f(0, Constants.movementStepSize, 0));
                    break;
                case Key.LeftShift:
                    camera.MoveBy(new Vector3f(0, -Constants.movementStepSize, 0));
                    break;
            }
        }
        public void HandleMouseInput(object sender, MouseEventArgs e)
        {
            System.Windows.Point currentMousePosition = Mouse.GetPosition(null);
            double deltaX = currentMousePosition.X - previousMousePosition.X;
            double deltaY = currentMousePosition.Y - previousMousePosition.Y;
            previousMousePosition = currentMousePosition;
            Vector3f deltaDirection = new Vector3f(
                (float)(deltaX * Constants.mouseSensitivity),
                (float)(-deltaY * Constants.mouseSensitivity),
                0
            );
            camera.TurnBy(deltaDirection);
        }
    }
    class World
    {
        private BoundingVolumeHierarchy boundingVolumeHirachy;
        public List<PointLight> lights;
        public World()
        {
            List<BoundingVolume> boundingVolumes = new List<BoundingVolume>();
            boundingVolumes.Add(new BoundingBox(new Vector3f(-6, -10, -6), new Vector3f(-3, 0, -3), new Material(new Color(0, 0, 200), 0.6f, 0.1f)));
            boundingVolumes.Add(new BoundingBox(new Vector3f(-10, -11, -10), new Vector3f(10, -10, 10), new Material(new Color(0, 200, 25))));
            boundingVolumes.Add(new BoundingBox(new Vector3f(-11, -11, -10), new Vector3f(-10, 0, 10), new Material(new Color(200, 200, 0), 1, 0.8f)));
            boundingVolumes.Add(new BoundingBox(new Vector3f(-10, -11, -11), new Vector3f(10, 0, -10), new Material(new Color(0, 200, 200))));
            boundingVolumes.Add(new BoundingBox(new Vector3f(-10, -11, 10), new Vector3f(10, 0, 11), new Material(new Color(0, 200, 0), 0.1f, 0)));
            boundingVolumes.Add(new BoundingBox(new Vector3f(10, -11, -10), new Vector3f(11, 0, 10), new Material(new Color(0, 200, 0))));
            boundingVolumes.Add(new BoundingSphere(new Vector3f(2, 2, 2), 1, new Material(new Color(255, 255, 0), 0.3f, 0.8f)));
            boundingVolumeHirachy = new BoundingVolumeHierarchy(boundingVolumes);
            lights = new List<PointLight>();
            lights.Add(new PointLight(new Vector3f(-8, 4, -4),1f));
            //lights.Add(new PointLight(new Vector3f(4, 10, 6), 0.2f));
        }
        // alternative initial values for bounding volumes and lights
        /*
        public World()
        {
            List<BoundingVolume> boundingVolumes = new List<BoundingVolume>();

            // Existing bounding boxes
            //boundingVolumes.Add(new BoundingBox(new Vector3f(-1, -10, -3), new Vector3f(1, 0, -1), new Material(new Color(0, 0, 200))));
            boundingVolumes.Add(new BoundingBox(new Vector3f(-10, -11, -10), new Vector3f(10, -10, 10), new Material(new Color(0, 200, 25))));
            boundingVolumes.Add(new BoundingBox(new Vector3f(-11, -11, -10), new Vector3f(-10, 0, 10), new Material(new Color(0, 200, 0))));
            boundingVolumes.Add(new BoundingBox(new Vector3f(-10, -11, -11), new Vector3f(10, 0, -10), new Material(new Color(0, 200, 0))));
            boundingVolumes.Add(new BoundingBox(new Vector3f(-10, -11, 10), new Vector3f(10, 0, 11), new Material(new Color(0, 200, 0))));
            boundingVolumes.Add(new BoundingBox(new Vector3f(10, -11, -10), new Vector3f(11, 0, 10), new Material(new Color(0, 200, 0))));
            // Seat
            boundingVolumes.Add(new BoundingBox(new Vector3f(-1, 0, -1), new Vector3f(1, 0.5f, 1), new Material(new Color(139, 69, 19)))); // Brown seat

            // Backrest
            boundingVolumes.Add(new BoundingBox(new Vector3f(-1, 0.5f, 0.8f), new Vector3f(1, 2, 1), new Material(new Color(139, 69, 19)))); // Brown backrest

            // Legs
            boundingVolumes.Add(new BoundingBox(new Vector3f(-0.9f, -2, -0.9f), new Vector3f(-0.7f, 0, -0.7f), new Material(new Color(105, 105, 105)))); // Front-left leg
            boundingVolumes.Add(new BoundingBox(new Vector3f(0.7f, -2, -0.9f), new Vector3f(0.9f, 0, -0.7f), new Material(new Color(105, 105, 105)))); // Front-right leg
            boundingVolumes.Add(new BoundingBox(new Vector3f(-0.9f, -2, 0.7f), new Vector3f(-0.7f, 0, 0.9f), new Material(new Color(105, 105, 105)))); // Back-left leg
            boundingVolumes.Add(new BoundingBox(new Vector3f(0.7f, -2, 0.7f), new Vector3f(0.9f, 0, 0.9f), new Material(new Color(105, 105, 105)))); // Back-right leg
            
            // New bounding boxes
            boundingVolumes.Add(new BoundingBox(new Vector3f(2, -5, -3), new Vector3f(5, 0, -1), new Material(new Color(200, 0, 0)))); // Red box
            boundingVolumes.Add(new BoundingBox(new Vector3f(3, -8, 4), new Vector3f(7, 0, 8), new Material(new Color(0, 0, 255)))); // Blue box
            boundingVolumes.Add(new BoundingBox(new Vector3f(-7, -15, 7), new Vector3f(-3, -10, 15), new Material(new Color(0, 255, 255)))); // Cyan box

            // Additional bounding boxes
            boundingVolumes.Add(new BoundingBox(new Vector3f(6, -4, -2), new Vector3f(9, 0, 2), new Material(new Color(255, 0, 0)))); // Red box
            boundingVolumes.Add(new BoundingBox(new Vector3f(12, -8, 3), new Vector3f(14, -3, 6), new Material(new Color(255, 165, 0)))); // Orange box
            boundingVolumes.Add(new BoundingBox(new Vector3f(-5, -12, -5), new Vector3f(0, -8, 5), new Material(new Color(255, 255, 255)))); // White box
            boundingVolumes.Add(new BoundingBox(new Vector3f(8, -7, 8), new Vector3f(10, -3, 10), new Material(new Color(0, 255, 0)))); // Green box
            boundingVolumes.Add(new BoundingBox(new Vector3f(-3, -9, -8), new Vector3f(0, -6, -6), new Material(new Color(255, 255, 0)))); // Yellow box
            boundingVolumes.Add(new BoundingBox(new Vector3f(4, -6, -5), new Vector3f(6, -1, -3), new Material(new Color(128, 0, 128)))); // Purple box
            
            boundingVolumeHirachy = new BoundingVolumeHierarchy(boundingVolumes);

            // Lights
            lights = new List<PointLight>();
            lights.Add(new PointLight(new Vector3f(-2, 4, -5), 0.3f)); // Existing light
            lights.Add(new PointLight(new Vector3f(6, 4, -3), 0.3f)); // Existing light
            lights.Add(new PointLight(new Vector3f(-2, 40, -5), 0.3f)); // Existing light
            lights.Add(new PointLight(new Vector3f(8, 2, 2), 0.3f)); // Existing light
        }
        */
        public void UpdateBoundingVolumesWithFrustrumCulling(Frustum frustrum)
        {
            boundingVolumeHirachy.UpdateFrustrumOfAllBoundingVolumes(frustrum);
        }
        public void MoveLights(float x, float y, float z)
        {
            for (int i = 0; i < lights.Count; i++) lights[i].position = lights[i].position + new Vector3f(x, y, z);
        }
        public bool CheckRayIntersection(Ray ray, out BoundingVolume closestVolume, out float closestDistance, out Vector3f pointOfCollision)
        {
            return boundingVolumeHirachy.GetIntersection(ray, out closestVolume, out closestDistance, out pointOfCollision, true);
        }
        public bool IntersectsAnything(Ray ray)
        {
            return boundingVolumeHirachy.IntersectsAnything(ray);
        }
        public void DoShadowCasting(ref Color initialColor, float shadowRayHits)
        {
            float shadowFactor = MathF.Min((shadowRayHits) / Constants.nShadowRays, 1);
            initialColor *= shadowFactor;
        }
        public float GetAmountOfShadowRayhits(Collision collision)
        {
            if (collision.boundingVolume is BoundingSphere)
                return Constants.nShadowRays;
            float nUnblockedRays = 0;
            foreach (var light in lights)
            {
                nUnblockedRays += light.GetNUnblockedShadowRays(collision, this);
            }
            return nUnblockedRays;
        }
    }
    class Frustum
    {
        private FrustumPlane near = new FrustumPlane();
        private FrustumPlane far = new FrustumPlane();
        private FrustumPlane right = new FrustumPlane();
        private FrustumPlane left = new FrustumPlane();
        private FrustumPlane top = new FrustumPlane();
        private FrustumPlane bottom = new FrustumPlane();
        public readonly List<FrustumPlane> planes;

        float tanFovHalf;
        float nearWidth;
        float nearHeight;

        public Frustum(float farPlaneDistance, float nearPlaneDistance)
        {
            tanFovHalf = MathF.Tan(Constants.fov / 2);
            nearWidth = 2 * nearPlaneDistance * tanFovHalf;
            nearHeight = nearWidth; 
            planes = new List<FrustumPlane>();
        }
        public void UpdatePlanes(Vector3f camPosition, Vector3f camDirection)
        {
            lock (this)
            {
                Vector3f constrainedForward = new Vector3f(camDirection.x, 0, camDirection.z).Normalize();
                Vector3f globalUp = new Vector3f(0, 1, 0);
                Vector3f rightDirection = Vector3f.Cross(globalUp, constrainedForward).Normalize();
                // calculate normals
                near.normal = camDirection;
                far.normal = -camDirection;
                left.normal = (camDirection - rightDirection * (nearWidth / 2)).Normalize();
                right.normal = (camDirection + rightDirection * (nearWidth / 2)).Normalize();
                top.normal = (camDirection + globalUp * (nearHeight / 2)).Normalize();
                bottom.normal = (camDirection - globalUp * (nearHeight / 2)).Normalize();
                // calculate points on plane
                near.pointOnPlane = camPosition + Constants.nearPlaneDistance * camDirection;
                far.pointOnPlane = camPosition + Constants.farPlaneDistance * camDirection;
                left.pointOnPlane = camPosition + Constants.nearPlaneDistance * camDirection - (nearWidth / 2) * rightDirection;
                right.pointOnPlane = camPosition + Constants.nearPlaneDistance * camDirection + (nearWidth / 2) * rightDirection;
                top.pointOnPlane = camPosition + Constants.nearPlaneDistance * camDirection + (nearHeight / 2) * globalUp;
                bottom.pointOnPlane = camPosition + Constants.nearPlaneDistance * camDirection - (nearHeight / 2) * globalUp;
                // Clear previous planes
                planes.Clear();
                planes.Add(near);
                planes.Add(far);
                planes.Add(left);
                planes.Add(right);
                planes.Add(top);
                planes.Add(bottom);
            }
        }
    }
    class FrustumPlane
    {
        public Vector3f normal;
        public Vector3f pointOnPlane;
    }
    class PointLight
    {
        public Vector3f position;
        public float intensity;
        public PointLight(Vector3f position, float intesity = 1f)
        {
            this.position = position;
            this.intensity = intesity;
        }
        public float GetNUnblockedShadowRays(Collision collision, World world)
        {
            float unblockedRays = 0;
            for (int i = 0; i < Constants.nShadowRays; i++)
            {
                Vector3f origin = collision.pointOfCollision + Vector3f.GetRandomDirectionInHalfSphere(collision.GetNormal()) * Constants.shadowSoftness;
                Vector3f shadowRayDir = (position - origin);
                Vector3f.Normalize(ref shadowRayDir);
                Ray shadowRay = new Ray(origin, shadowRayDir);
                if (!world.IntersectsAnything(shadowRay))
                    unblockedRays += intensity;
            }
            return unblockedRays;
        }
    }
    class BoundingVolumeHierarchy
    {
        private List<BoundingVolumeHierarchyNode> FirstLevelNodes;
        public BoundingVolumeHierarchy(List<BoundingVolume> objects)
        {
            FirstLevelNodes = new List<BoundingVolumeHierarchyNode>();
            if (objects == null || objects.Count == 0) return;
            Vector3f smallestMin = Vector3f.maxValue;
            Vector3f biggestMax = Vector3f.minValue;
            foreach (var obj in objects)
            {
                if (!obj.isVisibleInFrustrum) continue;
                obj.GetBoxArroundBoundingVolume(out Vector3f min, out Vector3f max);
                smallestMin = Vector3f.Min(min, smallestMin);
                biggestMax = Vector3f.Max(max, biggestMax);
            }
            FirstLevelNodes.Add(new BoundingVolumeHierarchyNode(smallestMin, biggestMax, objects));
        }
        public bool GetIntersection(Ray ray, out BoundingVolume closestVolume, out float closestDistance, out Vector3f pointOfCollision, bool doFrustrumCulling)
        {
            closestVolume = null;
            closestDistance = float.MaxValue;
            bool hit = false;
            pointOfCollision = Vector3f.Zero;
            foreach (var volume in FirstLevelNodes)
            {
                if (doFrustrumCulling)
                    if (!volume.isVisibleInFrustrum)
                        continue;
                if (volume.Intersects(ray, out float distance, out Vector3f intersectionPoint, true) && distance < closestDistance)
                {
                    if (volume is BoundingVolumeHierarchyNode)
                    {
                        if (volume.GetIntersection(ray, out BoundingVolume volumeFromHirachy, out distance, out intersectionPoint, true))
                        {
                            closestDistance = distance;
                            closestVolume = volumeFromHirachy;
                            hit = true;
                            pointOfCollision = intersectionPoint;
                        }
                    }
                    else
                    {
                        closestDistance = distance;
                        closestVolume = volume;
                        hit = true;
                        pointOfCollision = intersectionPoint;
                    }
                }
            }
            return hit;
        }
        public bool IntersectsAnything(Ray ray)
        {
            foreach (var volume in FirstLevelNodes)
            {
                if (volume.Intersects(ray, false))
                {
                    return true;
                }
            }
            return false;
        }
        public void UpdateFrustrumOfAllBoundingVolumes(Frustum frustrum)
        {
            foreach (var volume in FirstLevelNodes)
            {
                if (volume.IsInsideOfFrustrum(frustrum.planes))
                {
                    volume.isVisibleInFrustrum = true;
                    if (volume is BoundingVolumeHierarchyNode boundingVolumeHierachyNode)
                        boundingVolumeHierachyNode.UpdateFrustrumOfAllChildBoundingVolumes(frustrum);
                }
                else volume.isVisibleInFrustrum = false;
            }
        }
    }
    interface IVisible
    {
        public Vector3f GetExitPoint(Ray ray);
    }
    abstract class BoundingVolume
    {
        public bool isVisibleInFrustrum = true;
        public abstract bool Intersects(Ray ray, out float distance, out Vector3f intersectionPoint, bool doFrustrumCulling);
        public abstract bool Intersects(Ray ray, bool doFrustrumCulling);
        public abstract Vector3f GetNormalAtPoint(Vector3f point);
        public abstract void GetBoxArroundBoundingVolume(out Vector3f min, out Vector3f max);
        public Material material = new Material(new Color(0, 0, 255));
        public abstract bool IsInsideOfFrustrum(List<FrustumPlane> planes);
        public Color GetColor(Vector3f intersectionPoint, Vector3f viewDirection, List<PointLight> lights)
        {
            Vector3f normal = GetNormalAtPoint(intersectionPoint);
            Color finalColor = default;
            foreach (var light in lights)
            {
                // Calculate light direction 
                Vector3f lightDir = light.position - intersectionPoint;
                Vector3f.Normalize(ref lightDir);
                // Diffuse component
                float diffuseIntensity = MathF.Max(0, Vector3f.Dot(normal, lightDir)) * light.intensity;
                Color diffuseColor = material.color * diffuseIntensity * material.opacity;
                // Specular component 
                Vector3f reflectionDirection = 2 * Vector3f.Dot(normal, lightDir) * normal - lightDir;
                float specularIntensity = MathF.Pow(MathF.Max(0, Vector3f.Dot(reflectionDirection, viewDirection)), material.shininess) * light.intensity;
                Color specularColor = material.specularColor * specularIntensity;
                // Calculate final color
                finalColor += diffuseColor + specularColor;
            }
            return finalColor;
        }
    }
    class BoundingVolumeHierarchyNode : BoundingVolume
    {
        public Vector3f min;
        public Vector3f max;
        public List<BoundingVolume> boundingBoxes = new List<BoundingVolume>();
        public BoundingVolumeHierarchyNode(Vector3f min, Vector3f max, List<BoundingVolume> innerBoundingVolumes, bool doFrustrumCulling = false)
        {
            this.min = min;
            this.max = max;
            boundingBoxes.AddRange(innerBoundingVolumes);
        }
        public BoundingVolumeHierarchyNode(Vector3f min, Vector3f max)
        {
            this.min = min;
            this.max = max;
        }
        public override bool IsInsideOfFrustrum(List<FrustumPlane> planes)
        {
            foreach (var plane in planes)
            {
                Vector3f positiveCorner = new Vector3f(
                    plane.normal.x >= 0 ? max.x : min.x,
                    plane.normal.y >= 0 ? max.y : min.y,
                    plane.normal.z >= 0 ? max.z : min.z
                );
                Vector3f negativeCorner = new Vector3f(
                    plane.normal.x >= 0 ? min.x : max.x,
                    plane.normal.y >= 0 ? min.y : max.y,
                    plane.normal.z >= 0 ? min.z : max.z
                );
                float positiveDistance = Vector3f.Dot(plane.normal, positiveCorner) - Vector3f.Dot(plane.normal, plane.pointOnPlane);
                float negativeDistance = Vector3f.Dot(plane.normal, negativeCorner) - Vector3f.Dot(plane.normal, plane.pointOnPlane);
                if (positiveDistance > 0 || negativeDistance > 0)
                    continue;
                else
                    return false;
            }
            return true;
        }
        public void UpdateFrustrumOfAllChildBoundingVolumes(Frustum frustrum)
        {
            foreach (var volume in boundingBoxes)
            {
                if (volume.IsInsideOfFrustrum(frustrum.planes))
                {
                    volume.isVisibleInFrustrum = true;
                    if (volume is BoundingVolumeHierarchyNode boundingVolumeHierachyNode)
                        boundingVolumeHierachyNode.UpdateFrustrumOfAllChildBoundingVolumes(frustrum);
                }
                else volume.isVisibleInFrustrum = false;
            }
        }
        public override void GetBoxArroundBoundingVolume(out Vector3f min, out Vector3f max)
        {
            min = this.min;
            max = this.max;
        }
        public bool GetIntersection(Ray ray, out BoundingVolume closestVolume, out float closestDistance, out Vector3f pointOfCollision, bool doFrustrumCulling)
        {
            closestVolume = null;
            closestDistance = float.MaxValue;
            bool hit = false;
            pointOfCollision = Vector3f.Zero;
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            foreach (var volume in boundingBoxes)
            {
                if (doFrustrumCulling)
                    if (!volume.isVisibleInFrustrum)
                        continue;
                if (volume.Intersects(ray, out float distance, out Vector3f intersectionPoint, true) && distance < closestDistance)
                {
                    closestDistance = distance;
                    closestVolume = volume;
                    hit = true;
                    pointOfCollision = intersectionPoint;
                }
            }
            if (!hit) return false;
            if (closestVolume is BoundingVolumeHierarchyNode)
                ((BoundingVolumeHierarchyNode)closestVolume).GetIntersection(ray, out closestVolume, out closestDistance, out pointOfCollision, true);
            return true;
        }
        public override bool Intersects(Ray ray, out float distance, out Vector3f intersectionPoint, bool doFrustrumCulling)
        {
            distance = float.MaxValue;
            intersectionPoint = default(Vector3f);
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            float tMin = (min.x - ray.origin.x) / ray.direction.x;
            float tMax = (max.x - ray.origin.x) / ray.direction.x;
            if (tMin > tMax)
            {
                float temp = tMin;
                tMin = tMax;
                tMax = temp;
            }
            float tyMin = (min.y - ray.origin.y) / ray.direction.y;
            float tyMax = (max.y - ray.origin.y) / ray.direction.y;
            if (tyMin > tyMax)
            {
                float temp = tyMin;
                tyMin = tyMax;
                tyMax = temp;
            }
            if ((tMin > tyMax) || (tyMin > tMax))
                return false;
            tMin = MathF.Max(tMin, tyMin);
            tMax = MathF.Min(tMax, tyMax);
            float tzMin = (min.z - ray.origin.z) / ray.direction.z;
            float tzMax = (max.z - ray.origin.z) / ray.direction.z;
            if (tzMin > tzMax)
            {
                float temp = tzMin;
                tzMin = tzMax;
                tzMax = temp;
            }
            if ((tMin > tzMax) || (tzMin > tMax))
                return false;
            tMin = MathF.Max(tMin, tzMin);
            tMax = MathF.Min(tMax, tzMax);
            if (tMax < 0)
            {
                distance = float.MaxValue;
                return false;
            }
            distance = tMin >= 0 ? tMin : tMax;
            intersectionPoint = ray.origin + ray.direction * distance;
            return true;
        }
        public override Vector3f GetNormalAtPoint(Vector3f point)
        {
            Vector3f normal = default(Vector3f);
            float epsilon = 1e-3f;
            if (MathF.Abs(point.x - min.x) < epsilon) normal = new Vector3f(-1, 0, 0); // Left 
            else if (MathF.Abs(point.x - max.x) < epsilon) normal = new Vector3f(1, 0, 0); // Right 
            else if (MathF.Abs(point.y - min.y) < epsilon) normal = new Vector3f(0, -1, 0); // Bottom 
            else if (MathF.Abs(point.y - max.y) < epsilon) normal = new Vector3f(0, 1, 0); // Top 
            else if (MathF.Abs(point.z - min.z) < epsilon) normal = new Vector3f(0, 0, -1); // Back 
            else if (MathF.Abs(point.z - max.z) < epsilon) normal = new Vector3f(0, 0, 1); // Front 
            return normal;
        }
        public override bool Intersects(Ray ray, bool doFrustrumCulling)
        {
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            Vector3f invDir = new Vector3f(1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z);
            float tMin, tMax, tyMin, tyMax, tzMin, tzMax;
            // X-axis intersection
            if (invDir.x >= 0)
            {
                tMin = (min.x - ray.origin.x) * invDir.x;
                tMax = (max.x - ray.origin.x) * invDir.x;
            }
            else
            {
                tMin = (max.x - ray.origin.x) * invDir.x;
                tMax = (min.x - ray.origin.x) * invDir.x;
            }
            // Y-axis intersection
            if (invDir.y >= 0)
            {
                tyMin = (min.y - ray.origin.y) * invDir.y;
                tyMax = (max.y - ray.origin.y) * invDir.y;
            }
            else
            {
                tyMin = (max.y - ray.origin.y) * invDir.y;
                tyMax = (min.y - ray.origin.y) * invDir.y;
            }
            // Check for early exit on Y axis
            if ((tMin > tyMax) || (tyMin > tMax))
                return false;
            tMin = MathF.Max(tMin, tyMin);
            tMax = MathF.Min(tMax, tyMax);
            // Z-axis intersection
            if (invDir.z >= 0)
            {
                tzMin = (min.z - ray.origin.z) * invDir.z;
                tzMax = (max.z - ray.origin.z) * invDir.z;
            }
            else
            {
                tzMin = (max.z - ray.origin.z) * invDir.z;
                tzMax = (min.z - ray.origin.z) * invDir.z;
            }
            // Check for early exit on Z axis
            if ((tMin > tzMax) || (tzMin > tMax))
                return false;
            tMin = MathF.Max(tMin, tzMin);
            tMax = MathF.Min(tMax, tzMax);
            // Inner bounding volumes
            if (tMax < 0) return false;
            foreach (var boundingvolume in boundingBoxes)
                if (boundingvolume.Intersects(ray, doFrustrumCulling))
                    return true;
            return false;
        }
    }
    class Plane : BoundingVolume
    {
        private Vector3f pointOnPlane;
        private Vector3f normal;
        public Plane(Vector3f pointOnPlane, Vector3f normal, Material material)
        {
            this.pointOnPlane = pointOnPlane;
            this.normal = normal.Normalize();
            this.material = material;
        }
        public override bool IsInsideOfFrustrum(List<FrustumPlane> planes)
        {
            throw new NotImplementedException("This Volume is not cullable because of the undefined BoundingBox.");
        }
        public override void GetBoxArroundBoundingVolume(out Vector3f min, out Vector3f max)
        {
            throw new NotImplementedException("Bounding box for an infinite plane is not defined.");
        }
        public override Vector3f GetNormalAtPoint(Vector3f point)
        {
            return normal;
        }
        public override bool Intersects(Ray ray, out float distance, out Vector3f intersectionPoint, bool doFrustrumCulling)
        {
            intersectionPoint = default(Vector3f);
            distance = float.MaxValue;
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            float denominator = Vector3f.Dot(normal, ray.direction);
            if (MathF.Abs(denominator) < 1e-6)
                return false;
            float t = Vector3f.Dot(pointOnPlane - ray.origin, normal) / denominator;
            if (t < 0)
                return false;
            intersectionPoint = ray.origin + ray.direction * t;
            distance = t;
            return true;
        }
        public override bool Intersects(Ray ray, bool doFrustrumCulling)
        {
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            float denominator = Vector3f.Dot(normal, ray.direction);
            if (MathF.Abs(denominator) < 1e-6)
                return false; 
            float t = Vector3f.Dot(pointOnPlane - ray.origin, normal) / denominator;
            return t >= 0; 
        }
    }
    class BoundingSphere : BoundingVolume, IVisible
    {
        public Vector3f center;
        public float radius;
        public BoundingSphere(Vector3f center, float radius, Material material)
        {
            this.center = center;
            this.radius = radius;
            this.material = material;
        }
        public override bool IsInsideOfFrustrum(List<FrustumPlane> planes)
        {
            foreach (var plane in planes)
            {
                float distance = Vector3f.Dot(plane.normal, center) - Vector3f.Dot(plane.normal, plane.pointOnPlane);
                if (distance < -radius)
                {
                    return false;
                }
            }
            return true;
        }
        public override void GetBoxArroundBoundingVolume(out Vector3f min, out Vector3f max)
        {
            min = center - new Vector3f(radius, radius, radius);
            max = center + new Vector3f(radius, radius, radius);
        }
        public override bool Intersects(Ray ray, out float distance, out Vector3f intersectionPoint, bool doFrustrumCulling)
        {
            distance = float.MaxValue;
            intersectionPoint = default(Vector3f);
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            Vector3f toSphere = ray.origin - center;
            float a = Vector3f.Dot(ray.direction, ray.direction);
            float b = 2 * Vector3f.Dot(toSphere, ray.direction);
            float c = Vector3f.Dot(toSphere, toSphere) - (radius * radius);
            float discriminant = b * b - 4 * a * c;
            if (discriminant >= 0)
            {
                float sqrtDiscriminant = MathF.Sqrt(discriminant);
                float t1 = (-b - sqrtDiscriminant) / (2 * a);
                float t2 = (-b + sqrtDiscriminant) / (2 * a);
                if (t1 > 0)
                {
                    distance = t1;
                    intersectionPoint = ray.origin + ray.direction * t1;
                    return true;
                }
                else if (t2 > 0)
                {
                    distance = t2;
                    intersectionPoint = ray.origin + ray.direction * t2;
                    return true;
                }
            }
            intersectionPoint = default(Vector3f);
            return false;
        }
        public override Vector3f GetNormalAtPoint(Vector3f point)
        {
            return (point - center).Normalize();
        }
        public override bool Intersects(Ray ray, bool doFrustrumCulling)
        {
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            Vector3f toSphere = ray.origin - center;
            float a = Vector3f.Dot(ray.direction, ray.direction);
            float b = 2 * Vector3f.Dot(toSphere, ray.direction);
            float c = Vector3f.Dot(toSphere, toSphere) - (radius * radius);
            float discriminant = b * b - 4 * a * c;
            if (discriminant >= 0)
            {
                float sqrtDiscriminant = MathF.Sqrt(discriminant);
                float t1 = (-b - sqrtDiscriminant) / (2 * a);
                float t2 = (-b + sqrtDiscriminant) / (2 * a);
                if (t1 > 0 || t2 > 0)
                {
                    return true;
                }
            }
            return false;
        }
        public Vector3f GetExitPoint(Ray ray)
        {
            Vector3f center = this.center;
            float radius = this.radius;
            Vector3f oc = ray.origin - center;
            float a = Vector3f.Dot(ray.direction, ray.direction);
            float b = 2.0f * Vector3f.Dot(ray.direction, oc);
            float c = Vector3f.Dot(oc, oc) - radius * radius;
            float discriminant = b * b - 4 * a * c;
            float sqrtDiscriminant = MathF.Sqrt(discriminant);
            float t2 = (-b + sqrtDiscriminant) / (2.0f * a);
            Vector3f exitPoint = ray.origin + ray.direction * t2;
            return exitPoint;
        }
    }
    class BoundingBox : BoundingVolume, IVisible
    {
        public Vector3f min;
        public Vector3f max;
        public BoundingBox(Vector3f min, Vector3f max, Material material)
        {
            this.min = min;
            this.max = max;
            this.material = material;
        }
        public override string ToString()
        {
            return $"min x{min.x};min y{min.y};min z{min.z};max x{max.x};max y{max.y};max z{max.z}";
        }
        public override bool IsInsideOfFrustrum(List<FrustumPlane> planes)
        {
            foreach (var plane in planes)
            {
                Vector3f positiveCorner = new Vector3f(
                    plane.normal.x >= 0 ? max.x : min.x,
                    plane.normal.y >= 0 ? max.y : min.y,
                    plane.normal.z >= 0 ? max.z : min.z
                );
                Vector3f negativeCorner = new Vector3f(
                    plane.normal.x >= 0 ? min.x : max.x,
                    plane.normal.y >= 0 ? min.y : max.y,
                    plane.normal.z >= 0 ? min.z : max.z
                );
                float positiveDistance = Vector3f.Dot(plane.normal, positiveCorner) - Vector3f.Dot(plane.normal, plane.pointOnPlane);
                float negativeDistance = Vector3f.Dot(plane.normal, negativeCorner) - Vector3f.Dot(plane.normal, plane.pointOnPlane);
                if (positiveDistance > 0 || negativeDistance > 0)
                    continue;
                else
                    return false;
            }
            return true;
        }
        public Vector3f GetExitPoint(Ray ray)
        {
            Vector3f invDir = new Vector3f(1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z);
            float tMin, tMax, tyMin, tyMax, tzMin, tzMax;
            if (invDir.x >= 0)
            {
                tMin = (min.x - ray.origin.x) * invDir.x;
                tMax = (max.x - ray.origin.x) * invDir.x;
            }
            else
            {
                tMin = (max.x - ray.origin.x) * invDir.x;
                tMax = (min.x - ray.origin.x) * invDir.x;
            }
            if (invDir.y >= 0)
            {
                tyMin = (min.y - ray.origin.y) * invDir.y;
                tyMax = (max.y - ray.origin.y) * invDir.y;
            }
            else
            {
                tyMin = (max.y - ray.origin.y) * invDir.y;
                tyMax = (min.y - ray.origin.y) * invDir.y;
            }
            if ((tMin > tyMax) || (tyMin > tMax))
                return Vector3f.Zero;
            tMin = MathF.Max(tMin, tyMin);
            tMax = MathF.Min(tMax, tyMax);
            if (invDir.z >= 0)
            {
                tzMin = (min.z - ray.origin.z) * invDir.z;
                tzMax = (max.z - ray.origin.z) * invDir.z;
            }
            else
            {
                tzMin = (max.z - ray.origin.z) * invDir.z;
                tzMax = (min.z - ray.origin.z) * invDir.z;
            }
            if ((tMin > tzMax) || (tzMin > tMax))
                return Vector3f.Zero;
            tMin = MathF.Max(tMin, tzMin);
            tMax = MathF.Min(tMax, tzMax);
            if (tMax < 0)
                throw new InvalidOperationException("Ray does not intersect the box.");
            Vector3f exitPoint = ray.origin + ray.direction * tMax;
            return exitPoint;
        }
        public override void GetBoxArroundBoundingVolume(out Vector3f min, out Vector3f max)
        {
            min = this.min;
            max = this.max;
        }
        public override bool Intersects(Ray ray, out float distance, out Vector3f intersectionPoint, bool doFrustrumCulling)
        {
            distance = float.MaxValue;
            intersectionPoint = default(Vector3f);
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            float tMin = (min.x - ray.origin.x) / ray.direction.x;
            float tMax = (max.x - ray.origin.x) / ray.direction.x;
            if (tMin > tMax)
            {
                float temp = tMin;
                tMin = tMax;
                tMax = temp;
            }
            float tyMin = (min.y - ray.origin.y) / ray.direction.y;
            float tyMax = (max.y - ray.origin.y) / ray.direction.y;
            if (tyMin > tyMax)
            {
                float temp = tyMin;
                tyMin = tyMax;
                tyMax = temp;
            }
            if ((tMin > tyMax) || (tyMin > tMax))
                return false;
            tMin = MathF.Max(tMin, tyMin);
            tMax = MathF.Min(tMax, tyMax);
            float tzMin = (min.z - ray.origin.z) / ray.direction.z;
            float tzMax = (max.z - ray.origin.z) / ray.direction.z;
            if (tzMin > tzMax)
            {
                float temp = tzMin;
                tzMin = tzMax;
                tzMax = temp;
            }
            if ((tMin > tzMax) || (tzMin > tMax))
                return false;
            tMin = MathF.Max(tMin, tzMin);
            tMax = MathF.Min(tMax, tzMax);
            if (tMax < 0)
            {
                distance = float.MaxValue;
                return false;
            }
            distance = tMin >= 0 ? tMin : tMax; 
            intersectionPoint = ray.origin + ray.direction * distance;
            return true;
        }
        public override Vector3f GetNormalAtPoint(Vector3f point)
        {
            Vector3f normal = default(Vector3f);
            float epsilon = 1e-3f;
            if (MathF.Abs(point.x - min.x) < epsilon) normal = new Vector3f(-1, 0, 0); // Left 
            else if (MathF.Abs(point.x - max.x) < epsilon) normal = new Vector3f(1, 0, 0); // Right 
            else if (MathF.Abs(point.y - min.y) < epsilon) normal = new Vector3f(0, -1, 0); // Bottom 
            else if (MathF.Abs(point.y - max.y) < epsilon) normal = new Vector3f(0, 1, 0); // Top 
            else if (MathF.Abs(point.z - min.z) < epsilon) normal = new Vector3f(0, 0, -1); // Back 
            else if (MathF.Abs(point.z - max.z) < epsilon) normal = new Vector3f(0, 0, 1); // Front 
            return normal;
        }
        public override bool Intersects(Ray ray, bool doFrustrumCulling)
        {
            if (doFrustrumCulling)
                if (!isVisibleInFrustrum)
                    return false;
            Vector3f invDir = new Vector3f(1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z);
            float tMin, tMax, tyMin, tyMax, tzMin, tzMax;
            // X-axis intersection
            if (invDir.x >= 0)
            {
                tMin = (min.x - ray.origin.x) * invDir.x;
                tMax = (max.x - ray.origin.x) * invDir.x;
            }
            else
            {
                tMin = (max.x - ray.origin.x) * invDir.x;
                tMax = (min.x - ray.origin.x) * invDir.x;
            }
            // Y-axis intersection
            if (invDir.y >= 0)
            {
                tyMin = (min.y - ray.origin.y) * invDir.y;
                tyMax = (max.y - ray.origin.y) * invDir.y;
            }
            else
            {
                tyMin = (max.y - ray.origin.y) * invDir.y;
                tyMax = (min.y - ray.origin.y) * invDir.y;
            }
            // Check for early exit on Y axis
            if ((tMin > tyMax) || (tyMin > tMax))
                return false;
            tMin = MathF.Max(tMin, tyMin);
            tMax = MathF.Min(tMax, tyMax);
            // Z-axis intersection
            if (invDir.z >= 0)
            {
                tzMin = (min.z - ray.origin.z) * invDir.z;
                tzMax = (max.z - ray.origin.z) * invDir.z;
            }
            else
            {
                tzMin = (max.z - ray.origin.z) * invDir.z;
                tzMax = (min.z - ray.origin.z) * invDir.z;
            }
            // Check for early exit on Z axis
            if ((tMin > tzMax) || (tzMin > tMax))
                return false;
            tMin = MathF.Max(tMin, tzMin);
            tMax = MathF.Min(tMax, tzMax);
            // Inner bounding volumes
            if (tMax < 0) return false;
            return true;
        }
    }
    class Camera
    {
        private Vector3f position;
        private Vector3f backBufferPosition;
        private Vector3f direction;
        private Frustum frustum;
        private DirectionsPlane directionsPlane;
        private Color[,] image;
        private volatile bool hasTurned = false;
        private volatile bool hasMoved = false;
        public Camera(Vector3f position, Vector3f direction)
        {
            this.position = this.backBufferPosition = position;
            this.direction = direction.Normalize(); 
            directionsPlane = new DirectionsPlane();
            directionsPlane.Update(direction);
            frustum = new Frustum(50, 0.001f);
            image = new Color[Constants.resolution, Constants.resolution];
            frustum.UpdatePlanes(position, direction);
        }
        public void MoveBy(Vector3f deltaPosition)
        {
            backBufferPosition += ConvertDeltaDirectionToDeltaPosition(deltaPosition);
            hasMoved = true;
        }
        private Vector3f ConvertDeltaDirectionToDeltaPosition(Vector3f deltaDirection)
        {
            Vector3f constrainedForward = new Vector3f(direction.x, 0, direction.z).Normalize();
            Vector3f globalUp = new Vector3f(0, 1, 0);
            Vector3f right = Vector3f.Cross(globalUp, constrainedForward).Normalize();
            Vector3f deltaPosition =
                -right * deltaDirection.x +     
                globalUp * deltaDirection.y +  
                -constrainedForward * deltaDirection.z;
            return deltaPosition;
        }
        public void TurnBy(Vector3f deltaDirection)
        {
            direction += deltaDirection;
            Vector3f.Normalize(ref direction);
            hasTurned = true;
        }
        public void Draw(World world, MainWindow mainWindow)
        {
            //world.MoveLights(Constants.random.NextFloat(-1,1), 0, Constants.random.NextFloat(-1, 1));
            if (hasMoved || hasTurned)
            {
                frustum.UpdatePlanes(position, direction);
                world.UpdateBoundingVolumesWithFrustrumCulling(frustum);
            }
            if (hasTurned)
            {
                directionsPlane.Update(direction);
                hasTurned = false;
            }
            if (hasMoved)
            {
                position = backBufferPosition;
                hasMoved = false;
            }
            Parallel.For(0, Constants.resolution, x =>
            {
                for (int y = 0; y < Constants.resolution; y++)
                {
                    image[x, y] = new Ray(position, directionsPlane.directions[x, y]).GetColor(world, out float shadowRayHits, null);
                    if (!image[x, y].Equals(Constants.backGroundColor))
                        world.DoShadowCasting(ref image[x, y], shadowRayHits);
                }
            });
            if (Application.Current == null || mainWindow == null) return;
            Application.Current.Dispatcher.Invoke(new Action(() => mainWindow.DrawPixels(image)));
        }
    }
    class DirectionsPlane
    {
        public Vector3f[,] directions;
        private readonly float tanFovHalf;
        public DirectionsPlane()
        {
            this.tanFovHalf = MathF.Tan(MathF.PI * Constants.fov / 360f);
            directions = new Vector3f[Constants.resolution, Constants.resolution];
        }
        public void Update(Vector3f forward)
        {
            Vector3f worldUp = new Vector3f(0, 1, 0); // Assumes Y is up
            Vector3f right = Vector3f.Cross(forward, worldUp).Normalize();
            Vector3f up = Vector3f.Cross(right, forward).Normalize();
            float step = (2 * tanFovHalf) / (Constants.resolution - 1);
            for (int y = 0; y < Constants.resolution; y++)
            {
                for (int x = 0; x < Constants.resolution; x++)
                {
                    float offsetX = -tanFovHalf + x * step;
                    float offsetY = tanFovHalf - y * step;
                    Vector3f pointOnPlane = forward + right * offsetX + up * offsetY;
                    Vector3f.Normalize(ref pointOnPlane);
                    directions[x, y] = pointOnPlane;
                }
            }
        }
    }
    struct Ray
    {
        public Vector3f origin;
        public Vector3f direction;
        public Color color;
        public Ray(Vector3f origin, Vector3f direction, Color color)
        {
            this.origin = origin;
            this.direction = direction;
            this.color = color;
        }
        public Ray(Vector3f origin, Vector3f direction)
        {
            this.origin = origin;
            this.direction = direction;
            this.color = new Color(0, 0, 0);
        }
        private Ray CreateDiffusionRay(Collision collision)
        {
            return new Ray(collision.pointOfCollision + collision.GetNormal() * 1e-3f, Vector3f.GetRandomDirectionInHalfSphere(collision.GetNormal()), default);//color);
        }
        private Ray CreateReflectionRay(Collision collision)
        {
            return new Ray(collision.pointOfCollision + collision.GetNormal() * 2f, Vector3f.GetReflectanceDirection(direction, collision.GetNormal()), default);
        }
        private Ray CreateTransmittedRay(Collision collision, Vector3f refractedDirection)
        {
            Vector3f newOrigin = ((IVisible)collision.boundingVolume).GetExitPoint(new Ray(collision.pointOfCollision - refractedDirection * 1e-3f, refractedDirection)) + refractedDirection * 1e-3f;
            return new Ray(newOrigin, refractedDirection);
        }
        public Color GetColor(World world, out float shadowRayHits, BoundingVolume? previousBox, int iteration = 0)
        {
            shadowRayHits = 0;
            float attenuationFactor = 1f / ((iteration + 1));
            if (!GetNextCollision(world, out Collision collision))
                return Constants.backGroundColor * attenuationFactor;
            float reflectionCoefficient = collision.boundingVolume.material.GetFresnelReflection(direction, collision.GetNormal()) * collision.boundingVolume.material.reflectiveness;
            color += collision.boundingVolume.GetColor(collision.pointOfCollision, direction, world.lights) * attenuationFactor * (1 - reflectionCoefficient);
            shadowRayHits += world.GetAmountOfShadowRayhits(collision);
            if (iteration == Constants.maxRayBounceIterations)
                return color;
            if (collision.boundingVolume.material.reflectiveness != 0)
            {
                color += CreateReflectionRay(collision).GetColor(world, out float shadowRayHitsReflection, collision.boundingVolume, iteration + 1) * reflectionCoefficient;
                shadowRayHits += shadowRayHitsReflection * reflectionCoefficient;
            }
            if (collision.boundingVolume.material.opacity != 1)
            {
                Vector3f refractedDirection = Vector3f.GetRefractedDirection(direction, collision.GetNormal(), Constants.refractiveIndex, collision.boundingVolume.material.refractiveIndex);
                if (!refractedDirection.Equals(Vector3f.Zero))
                {
                    color += CreateTransmittedRay(collision, refractedDirection).GetColor(world, out float shadowRayHitsTransmission, collision.boundingVolume, iteration) * (1 - collision.boundingVolume.material.opacity);
                    shadowRayHits += shadowRayHitsTransmission;
                }
                else
                {
                    color += CreateReflectionRay(collision).GetColor(world, out float shadowRayHitsReflection, collision.boundingVolume, iteration + 1);
                    shadowRayHits += shadowRayHitsReflection;
                }
            }
            color += GetAccumulatedColor(iteration, collision, world, out float shadowRayHitsDiffusionRay);
            shadowRayHits += shadowRayHitsDiffusionRay * (1 - collision.boundingVolume.material.opacity);
            return color;
        }
        private bool GetNextCollision(World world, out Collision collision)
        {
            if (world.CheckRayIntersection(this, out BoundingVolume closestVolume, out float closestDistance, out Vector3f pointOfCollision))
            {
                collision = new Collision(closestVolume, closestDistance, pointOfCollision);
                return true;
            }
            collision = default;
            return false;
        }
        private Color GetAccumulatedColor(int iteration, Collision collision, World world, out float shadowRayHits)
        {
            shadowRayHits = 0;
            Color accumulatedColor = new Color(0, 0, 0);
            for (int i = 0; i < Constants.maxRayDiffusionAmount; i++)
            {
                accumulatedColor += CreateDiffusionRay(collision).GetColor(world, out float shadowRayHitsDiffusionRay, collision.boundingVolume, iteration + 1) / Constants.maxRayDiffusionAmount;
                shadowRayHits += shadowRayHitsDiffusionRay;
            }
            if (Constants.maxRayDiffusionAmount != 0)
                shadowRayHits /= Constants.maxRayDiffusionAmount;
            return accumulatedColor;
        }
    }
    struct Collision
    {
        public BoundingVolume boundingVolume;
        public float distance;
        public Vector3f pointOfCollision;
        public Collision(BoundingVolume boundingVolume, float distance, Vector3f pointOfCollision)
        {
            this.boundingVolume = boundingVolume;
            this.distance = distance;
            this.pointOfCollision = pointOfCollision;
        }
        public Vector3f GetNormal()
        {
            return boundingVolume.GetNormalAtPoint(pointOfCollision);
        }
    }
    public struct Color
    {
        private uint packedValue;
        public Color(byte r, byte g, byte b, byte a = 255)
        {
            packedValue = (uint)(r << 24 | g << 16 | b << 8 | a);
        }
        public byte r => (byte)(packedValue >> 24);
        public byte g => (byte)(packedValue >> 16);
        public byte b => (byte)(packedValue >> 8);
        public byte a => (byte)(packedValue);
        public (float, float, float) ToFloatComponents()
        {
            return (r / 255f, g / 255f, b / 255f);
        }
        public static Color operator +(Color c, Color d)
        {
            // Clamp the values to ensure they stay within the range of 0-255
            byte r = (byte)Math.Clamp(c.r + d.r, 0, 255);
            byte g = (byte)Math.Clamp(c.g + d.g, 0, 255);
            byte b = (byte)Math.Clamp(c.b + d.b, 0, 255);
            return new Color(r, g, b);
        }
        public static Color operator *(Color color, float intensity)
        {
            byte r = (byte)Math.Clamp(color.r * intensity, 0, 255);
            byte g = (byte)Math.Clamp(color.g * intensity, 0, 255);
            byte b = (byte)Math.Clamp(color.b * intensity, 0, 255);
            byte a = color.a;
            return new Color(r, g, b, a);
        }
        public static Color operator /(Color color, float scalar)
        {
            if (scalar == 0) return new Color(color.r, color.g, color.b, color.a);
            byte r = (byte)Math.Clamp(color.r / scalar, 0, 255);
            byte g = (byte)Math.Clamp(color.g / scalar, 0, 255);
            byte b = (byte)Math.Clamp(color.b / scalar, 0, 255);
            byte a = color.a;
            return new Color(r, g, b, a);
        }
        public static Color Lerp(Color start, Color end, float t)
        {
            t = Math.Clamp(t, 0f, 1f);
            byte r = (byte)(start.r + (end.r - start.r) * t);
            byte g = (byte)(start.g + (end.g - start.g) * t);
            byte b = (byte)(start.b + (end.b - start.b) * t);
            byte a = (byte)(start.a + (end.a - start.a) * t);
            return new Color(r, g, b, a);
        }
        public override string ToString()
        {
            return r.ToString() + "," + g.ToString() + "," + b.ToString();
        }
    }
    struct Vector3f
    {
        public float x;
        public float y;
        public float z;
        public static readonly Vector3f Zero = new Vector3f(0, 0, 0);
        public static readonly Vector3f minValue = new Vector3f(float.MinValue, float.MinValue, float.MinValue);
        public static readonly Vector3f maxValue = new Vector3f(float.MaxValue, float.MaxValue, float.MaxValue);
        public Vector3f(float x, float y, float z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public static Vector3f operator -(Vector3f a)
        {
            return new Vector3f(-a.x, -a.y, -a.z);
        }
        public static Vector3f operator +(Vector3f a, Vector3f b)
        {
            return new Vector3f(a.x + b.x, a.y + b.y, a.z + b.z);
        }
        public static Vector3f operator -(Vector3f a, Vector3f b)
        {
            return new Vector3f(a.x - b.x, a.y - b.y, a.z - b.z);
        }
        public static Vector3f operator *(Vector3f a, float scalar)
        {
            return new Vector3f(a.x * scalar, a.y * scalar, a.z * scalar);
        }
        public static Vector3f operator *(float scalar, Vector3f a)
        {
            return a * scalar;
        }
        public static Vector3f operator /(Vector3f a, float scalar)
        {
            float invScalar = 1.0f / scalar;
            return new Vector3f(a.x * invScalar, a.y * invScalar, a.z * invScalar);
        }
        public static float Distance(Vector3f a, Vector3f b)
        {
            float dx = a.x - b.x;
            float dy = a.y - b.y;
            float dz = a.z - b.z;
            return MathF.Sqrt(dx * dx + dy * dy + dz * dz);
        }
        public static float Dot(Vector3f a, Vector3f b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }
        public static Vector3f Min(Vector3f a, Vector3f b)
        {
            return new Vector3f(MathF.Min(a.x, b.x), MathF.Min(a.y, b.y), MathF.Min(a.z, b.z));
        }
        public static Vector3f Max(Vector3f a, Vector3f b)
        {
            return new Vector3f(MathF.Max(a.x, b.x), MathF.Max(a.y, b.y), MathF.Max(a.z, b.z));
        }
        public static Vector3f Cross(Vector3f a, Vector3f b)
        {
            return new Vector3f(
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            );
        }
        public float LengthSquared()
        {
            return x * x + y * y + z * z;
        }
        public float Magnitude()
        {
            return (float)Math.Sqrt(x * x + y * y + z * z);
        }
        public Vector3f Normalize()
        {
            float mag = Magnitude();
            return mag > 0 ? this / mag : new Vector3f(0, 0, 0);
        }
        public static void Normalize(ref Vector3f vector)
        {
            float magnitude = vector.Magnitude();
            if (magnitude > 0)
            {
                vector.x /= magnitude;
                vector.y /= magnitude;
                vector.z /= magnitude;
            }
        }
        public override string ToString()
        {
            return $"({x}, {y}, {z})";
        }
        public override bool Equals(object obj)
        {
            if (!(obj is Vector3f)) return false;
            Vector3f v = (Vector3f)obj;
            return x == v.x && y == v.y && z == v.z;
        }
        public override int GetHashCode()
        {
            return HashCode.Combine(x, y, z);
        }
        public static Vector3f GetRandomDirectionInHalfSphere(Vector3f normal)
        {
            Vector3f randomDirection;
            do
            {
                double theta = 2 * Math.PI * Constants.random.NextFloat(); 
                double phi = Math.Acos(2 * Constants.random.NextFloat() - 1); 
                double x = Math.Sin(phi) * Math.Cos(theta);
                double y = Math.Sin(phi) * Math.Sin(theta);
                double z = Math.Cos(phi);
                randomDirection = new Vector3f((float)x, (float)y, (float)z);
            }
            while (Vector3f.Dot(randomDirection, normal) < 0); 
            return randomDirection.Normalize(); 
        }
        public static Vector3f GetReflectanceDirection(Vector3f incidentDirection, Vector3f normal)
        {
            float dotProduct = Vector3f.Dot(incidentDirection, normal);
            return incidentDirection - 2 * dotProduct * normal;
        }
        public static Vector3f GetRefractedDirection(Vector3f incidentDirection, Vector3f normal, float refractiveIndexFrom, float refractiveIndexTo)
        {
            float eta = refractiveIndexFrom / refractiveIndexTo;
            float cosThetaI = -Vector3f.Dot(normal, incidentDirection);
            if (cosThetaI < 0)
            {
                cosThetaI = -cosThetaI;
                normal = -normal; 
                eta = refractiveIndexTo / refractiveIndexFrom;
            }
            // Snell's Law
            float sinThetaTSquared = eta * eta * (1 - cosThetaI * cosThetaI);
            if (sinThetaTSquared > 1.0f)
                return Vector3f.Zero;
            float cosThetaT = MathF.Sqrt(1.0f - sinThetaTSquared);
            return eta * incidentDirection + (eta * cosThetaI - cosThetaT) * normal;
        }
    }
    struct Material
    {
        // Diffuse properties
        public Color color;           // Base color of the material
        public float opacity;         // Opacity of the material (0 = transparent, 1 = opaque)
        // Specular properties
        public Color specularColor;   // Color of the specular reflection
        public float shininess;       // Controls the sharpness of specular highlights
        // Optical properties (Reflection and Refraction)
        public float refractiveIndex; // Refractive index of the material (used for refraction calculations)
        public float reflectiveness;  // How much of the light is reflected (1 = mirror like behaviour, 0 = no light gets reflected)
        public Material(Color color, float opacity = 1f, float reflectiveness = 0f, Color? specularColor = null, float shininess = 32f, float refractiveIndex = 1.05f)
        {
            this.color = color;
            this.opacity = opacity;
            this.specularColor = specularColor ?? new Color(255, 255, 255); 
            this.shininess = shininess;
            this.refractiveIndex = refractiveIndex;
            this.reflectiveness = reflectiveness;
        }
        public float GetFresnelReflection(Vector3f incidentDirection, Vector3f normal)
        {
            float cosTheta = Math.Max(Vector3f.Dot(incidentDirection, normal), 0f);
            float r0 = (1f - refractiveIndex) / (1f + refractiveIndex);
            r0 = r0 * r0;
            return r0 + (1 - r0) * (float)Math.Pow(1 - cosTheta, 5);
        }
    }
    public class Xorshift
    {
        private uint state;
        public Xorshift(uint seed)
        {
            state = seed;
        }
        public uint Next()
        {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            return state;
        }
        public float NextFloat()
        {
            return Next() / (float)uint.MaxValue;
        }
        public float NextFloat(float min, float max)
        {
            return NextFloat() * (max - min) + min;
        }
    }
    class Consoles
    {
        private MainWindow mainWindow;
        private Stack<(Delegate, object[] parameters)> stack = new Stack<(Delegate, object[] parameters)>();
        private static TextBox InputConsole = new TextBox
        {
            Name = "InputConsole",
            Width = 500,
            Background = Brushes.LightGray,
            Margin = new Thickness(5),
            IsReadOnly = false,
            AcceptsReturn = true,
            TextWrapping = TextWrapping.Wrap,
            VerticalScrollBarVisibility = ScrollBarVisibility.Auto
        };
        private static TextBox OutputConsole = new TextBox
        {
            Name = "OutputConsole",
            Width = 500,
            Background = Brushes.LightGray,
            Margin = new Thickness(5),
            IsReadOnly = true,
            IsEnabled = false,
            AcceptsReturn = true,
            TextWrapping = TextWrapping.Wrap,
            VerticalScrollBarVisibility = ScrollBarVisibility.Auto
        };
        public Consoles(MainWindow mainWindow)
        {
            this.mainWindow = mainWindow;
            mainWindow.ConsolesStackPanel.Children.Add(InputConsole);
            mainWindow.ConsolesStackPanel.Children.Add(OutputConsole);
            InputConsole.PreviewKeyDown += mainWindow.OnConsoleKeyInput;
        }
        public void ExecuteStack(World world)
        {
            if (stack.Count == 0)
                return;
            (Delegate, object[] parameters) command = stack.Pop();
            if (command.Item1 is Action action)
                action();
            else if (command.Item1 is Action<MainWindow> actionWithParam)
                actionWithParam(mainWindow);
            else if (command.Item2 != null)
                command.Item1.DynamicInvoke(FillWithStaticInstances(command.Item1, command.Item2, world));
            else
                throw new InvalidOperationException("Unsupported command type in stack.");
        }
        public void WriteLine(string text)
        {
            Application.Current.Dispatcher.BeginInvoke(new Action(() => {
                OutputConsole.AppendText($"{text}\n");
                OutputConsole.ScrollToEnd();
            }));
        }
        public void Write(string text)
        {
            Application.Current.Dispatcher.BeginInvoke(new Action(() => {
                OutputConsole.AppendText(text);
                OutputConsole.ScrollToEnd();
            }));
        }
        public void ParseInput()
        {
            string line;
            if ((line = GetLine()).Length < 2) return;
            if (line[0] != '/') return;
            (Delegate, object[])? command = ParseCommand(line);
            if (command.HasValue) stack.Push(command.GetValueOrDefault());
        }
        private string GetLine()
        {
            int caretIndex = InputConsole.CaretIndex;
            int lineIndex = InputConsole.GetLineIndexFromCharacterIndex(caretIndex);
            string lineText = InputConsole.GetLineText(lineIndex);
            return lineText;
        }
        private (Delegate, object[])? ParseCommand(string command)
        {
            try
            {
                int parethesesIndex;
                Delegate @delegate;
                int nParameters;
                if ((parethesesIndex = command.IndexOf('(')) == -1)
                    parethesesIndex = command.Length;
                commands.TryGetValue(command.Substring(0, parethesesIndex), out @delegate);
                if (@delegate == null) return null;
                nParameters = @delegate.Method.GetParameters().Length;
                if (parethesesIndex != command.Length)
                    return (@delegate, ParseParameters(command.Substring(parethesesIndex, command.Length - parethesesIndex)));
                return (@delegate, null);
            }
            catch (Exception e)
            {
                return null;
            }
        }
        private object[] FillWithStaticInstances(Delegate @delegate, object[] args, World world)
        {
            if (@delegate.Method.GetParameters().Length == args.Length) return args;
            object[] args2 = new object[@delegate.Method.GetParameters().Length];
            for (int i = 0; i < @delegate.Method.GetParameters().Length; i++)
            {
                var paramType = @delegate.Method.GetParameters()[i].ParameterType;
                if (paramType == typeof(MainWindow))
                {
                    args2[i] = mainWindow;
                }
                else if (paramType == typeof(float))
                {
                    if (i < args.Length)
                    {
                        args2[i] = args[i];
                    }
                    else
                    {
                        args2[i] = default(float);
                    }
                }
                else if (paramType == typeof(int))
                {
                    if (i < args.Length)
                    {
                        args2[i] = args[i];
                    }
                    else
                    {
                        args2[i] = default(int);
                    }
                }
                else if (paramType == typeof(World))
                {
                    args2[i] = world;
                }
                else
                {
                    throw new InvalidOperationException($"Unsupported parameter type: {paramType}");
                }
            }
            return args2;
        }
        private object[] ParseParameters(string str)
        {
            string[] parameters = str.Substring(1, str.Length - 2).Split(',');
            object[] args = new object[parameters.Length];
            for (int i = 0; i < parameters.Length; i++)
            {
                if (float.TryParse(parameters[i], out float value))
                    args[i] = value;
                else
                    return null;
            }
            return args;
        }
        private static Dictionary<string, Delegate> commands = new Dictionary<string, Delegate>()
        {
            {"/BackgroundRed",new Action(() => Constants.backGroundColor = new Color(0, 0, 255))},
            {"/BackgroundBlue",new Action(() => Constants.backGroundColor = new Color(255, 0, 0))},
            {"/BackgroundBlack",new Action(() => Constants.backGroundColor = new Color(0, 0, 0))},
            {"/SetBackgroundColor",new Action<float,float,float>((x,y,z) =>
            {
                Constants.backGroundColor = new Color((byte)x, (byte)y, (byte)z);
            })},
            {"/Exit",new Action<MainWindow>((MainWindow mainWindow) =>
            {
                Application.Current.Dispatcher.Invoke(new Action(() =>
                {
                    if (InputConsole.Parent is Panel parentPanel1)
                        parentPanel1.Children.Remove(InputConsole);
                    if (OutputConsole.Parent is Panel parentPanel2)
                        parentPanel2.Children.Remove(OutputConsole);
                    mainWindow.Width -= 800;
                    mainWindow.Height += 200;
                }));
            })},
            {"/Resize",new Action<float, float, MainWindow>((float x, float y, MainWindow mainWindow) =>
            {
                Application.Current.Dispatcher.Invoke(new Action(() =>
                {
                    mainWindow.Width = x;
                    mainWindow.Height = y;
                }));
            })},
            {"/MoveLight",new Action<float, float,float, World>((float x, float y, float z, World world) =>
            {
                world.MoveLights(x,y,z);
            })},
            {"/Help",new Action(() => Help())},
        };
        private static void Help()
        {
            StringBuilder commandKeys = new StringBuilder();
            foreach (var command in commands) commandKeys.Append(command.Key.ToString() + "   parameters:" + command.Value.Method.GetParameters().Length + "\n");
            MessageBox.Show(commandKeys.ToString(), "Help", MessageBoxButton.OK, MessageBoxImage.Information);
        }
        public void DisableInput()
        {
            InputConsole.IsReadOnly = true;
            InputConsole.IsEnabled = false;
        }
        public void EnableInput()
        {
            InputConsole.IsReadOnly = false;
            InputConsole.IsEnabled = true;
        }
    }
    static class Constants
    {
        public const int maxRayBounceIterations = 10;
        public const int maxRayDiffusionAmount =0;
        public const int nShadowRays = 100;
        public const int fps = 100;
        public const float fov = 90;
        public const int resolution = 1000;
        public const float mouseSensitivity = 0.003f;
        public const float movementStepSize = 0.1f;
        public const float farPlaneDistance = 1000;
        public const float nearPlaneDistance = 2;
        public const float shadowSoftness = 0.05f;
        public const float refractiveIndex = 1;
        public static readonly Xorshift random = new Xorshift(10);
        public static int totalNumberOfRays => (int)Math.Pow(maxRayDiffusionAmount, maxRayBounceIterations);
        public static Color backGroundColor = new Color(255, 0, 0);
        public static Consoles console;
    }
}
