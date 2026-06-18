using System;
using System.IO;
using System.Collections.Generic;
using System.Globalization;
using System.Diagnostics;
using System.Linq;
using RobotOM;

// ═══════════════════════════════════════════════════════════════════
//  RWIND → Robot RSA  |  Pressure CSV → Panel nodal forces
//
//  Canopy geometry (El Menia):
//    Length  48 m  → Robot Y axis  (0 → 48)
//    Width    6 m  → Robot X axis  (0 →  6)
//    Column   2.8 m height
//    Beam     3.8 m height (free end, roof inclined)
//
//  RWIND CSV format (tab-separated):
//    Col 0: Res_Surface_P  [Pa]
//    Col 1: Points:0       X_rwind  (width,  centered: –3 → +3)
//    Col 2: Points:1       Y_rwind  (length, centered: –24 → +24)
//    Col 3: Points:2       Z_rwind  (height, same as Robot Z)
//
//  Coordinate transformation RWIND → Robot:
//    X_robot = X_rwind  + offset_X      (offset_X  = half width  ≈ 3.0)
//    Y_robot = Y_rwind  + offset_Y      (offset_Y  = half length ≈ 24.0)
//    Z_robot = Z_rwind                  (no change)
//
//  Offsets are AUTO-DETECTED from CSV min values so no hardcoding.
// ═══════════════════════════════════════════════════════════════════

namespace RwindToRobot
{
    // ── Data structures ──────────────────────────────────────────────
    class RwindPoint
    {
        public double Xr, Yr, Zr;   // RWIND coordinates
        public double Xrobot, Yrobot, Zrobot; // Robot coordinates (after transform)
        public double Pressure_Pa;
        public double Pressure_kNm2 => Pressure_Pa / 1000.0;
    }

    class RoboNode
    {
        public int    Id;
        public double X, Y, Z;
    }

    // ── Entry point ───────────────────────────────────────────────────
    class Program
    {
        // ── Configuration ─────────────────────────────────────────────
        const string CSV_PATH =
            @"D:\lagh-univ2025-2026\master thesis\CSVresults\open canopy\+X48m.csv";

        const string CASE_NAME    = "Wind_RWIND_+X";
        const double MIN_PRESSURE = 0.01;   // kN/m² — skip near-zero cells
        const double MESH_SIZE    = 0.5;    // m — FE mesh target size on roof panel
        const double SEARCH_R     = 1.5;    // m — max dist RWIND point → Robot node

        // Canopy geometry (used only if auto-detect fails)
        const double CANOPY_WIDTH  = 6.0;   // m  (Robot X)
        const double CANOPY_LENGTH = 48.0;  // m  (Robot Y)
        // ──────────────────────────────────────────────────────────────

        static void Main(string[] args)
        {
            Console.OutputEncoding = System.Text.Encoding.UTF8;
            var totalTimer = Stopwatch.StartNew();

            Banner("RWIND → Robot RSA  |  Panel pressure load");

            // ── Step 1 : Read CSV ─────────────────────────────────────
            Step(1, "Reading RWIND CSV");
            string csvPath = args.Length > 0 ? args[0] : CSV_PATH;
            Info($"File: {csvPath}");

            var points = ReadCsv(csvPath);
            if (points.Count == 0) { Fatal("No data in CSV."); return; }
            OK($"{points.Count} pressure points loaded");

            // ── Step 2 : Compute coordinate offsets ───────────────────
            Step(2, "Computing coordinate transformation (RWIND → Robot)");

            double xMin = points.Min(p => p.Xr);
            double yMin = points.Min(p => p.Yr);
            double zMin = points.Min(p => p.Zr);

            // RWIND domain is centered → offset = -min  (so min maps to 0)
            double offsetX = -xMin;  // ≈ +3.0 m
            double offsetY = -yMin;  // ≈ +24.0 m
            double offsetZ = -zMin;  // ≈ 0 (Z already starts at 0)

            Info($"Auto-detected offsets:  ΔX={offsetX:F3}  ΔY={offsetY:F3}  ΔZ={offsetZ:F3}");

            // Apply transformation to every point
            foreach (var p in points)
            {
                p.Xrobot = p.Xr + offsetX;
                p.Yrobot = p.Yr + offsetY;
                p.Zrobot = p.Zr + offsetZ;
            }

            // Sanity check: print transformed bounding box
            Info($"Transformed X: [{points.Min(p=>p.Xrobot):F2} → {points.Max(p=>p.Xrobot):F2}]  " +
                 $"(expect 0 → {CANOPY_WIDTH})");
            Info($"Transformed Y: [{points.Min(p=>p.Yrobot):F2} → {points.Max(p=>p.Yrobot):F2}]  " +
                 $"(expect 0 → {CANOPY_LENGTH})");
            Info($"Transformed Z: [{points.Min(p=>p.Zrobot):F2} → {points.Max(p=>p.Zrobot):F2}]");
            OK("Transformation applied");

            // ── Step 3 : Connect to Robot ─────────────────────────────
            Step(3, "Connecting to Robot RSA");
            RobotApplication robot;
            try { robot = new RobotApplication(); robot.Visible = -1; }
            catch (Exception ex) { Fatal($"Cannot connect to Robot: {ex.Message}"); return; }

            IRobotProject   project   = robot.Project;
            IRobotStructure structure = project.Structure;

            if (project.IsActive == 0) { Fatal("No active project open in Robot."); return; }
            OK("Connected");

            // ── Step 4 : Load Robot nodes ─────────────────────────────
            Step(4, "Loading Robot nodes");
            var nodeServer  = (IRobotNodeServer)structure.Nodes;
            var allNodes    = nodeServer.GetAll();
            var robotNodes  = new List<RoboNode>(allNodes.Count);

            for (int i = 1; i <= allNodes.Count; i++)
            {
                var n = (IRobotNode)allNodes.Get(i);
                if (n != null)
                    robotNodes.Add(new RoboNode { Id = n.Number, X = n.X, Y = n.Y, Z = n.Z });
            }

            if (robotNodes.Count == 0) { Fatal("No nodes found in Robot model."); return; }
            OK($"{robotNodes.Count} nodes loaded");

            // ── Step 5 : Filter roof nodes only ──────────────────────
            Step(5, "Filtering roof nodes (Z ≥ 2.5 m)");

            // Roof nodes are those at beam height — above columns
            // Use Z ≥ 2.5 m as threshold (columns are 2.8 m, beams 3.8 m)
            double zRoofMin = 2.5;
            var roofNodes = robotNodes.Where(n => n.Z >= zRoofMin).ToList();

            if (roofNodes.Count == 0)
            {
                Warn("No nodes found above Z=2.5m — using all nodes instead.");
                roofNodes = robotNodes;
            }

            OK($"{roofNodes.Count} roof nodes identified");

            // ── Step 6 : Build spatial grid for fast lookup ───────────
            Step(6, "Building spatial lookup grid");
            double gridSize = EstimateGridSize(roofNodes);
            Info($"Grid cell size: {gridSize:F3} m");
            var grid = BuildGrid(roofNodes, gridSize);
            OK("Grid ready");

            // ── Step 7 : Estimate RWIND cell area ─────────────────────
            Step(7, "Estimating RWIND cell area");
            double cellArea = EstimateCellArea(points);
            Info($"Cell area: {cellArea:F4} m²");
            OK("Cell area estimated");

            // ── Step 8 : Create Robot load case ──────────────────────
            Step(8, $"Creating load case \"{CASE_NAME}\"");
            var caseServer  = (IRobotCaseServer)structure.Cases;
            int caseNum     = caseServer.GetAll().Count + 1;

            var simpleCase = (IRobotSimpleCase)caseServer.CreateSimple(
                caseNum,
                CASE_NAME,
                IRobotCaseNature.I_CN_WIND,
                IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);

            OK($"Load case #{caseNum} created");

            // ── Step 9 : Map pressures to Robot nodes ─────────────────
            Step(9, "Mapping RWIND pressures → Robot nodes");

            // Accumulate forces per Robot node
            // Force on node = sum of (pressure × cellArea) for all RWIND points
            // whose nearest Robot node is this node.
            // Direction = normal to roof surface (computed from roof geometry)
            var nodeForces = new Dictionary<int, (double Fx, double Fy, double Fz)>();

            // Pre-compute roof normal (outward = upward for a flat/inclined canopy)
            // For a canopy: pressure positive → pushes DOWN on top surface
            //               pressure negative → suction → pulls UP
            // Normal direction: (0, 0, 1) pointing upward (global Z)
            // Refined for inclined roof: compute from actual node positions
            (double nx, double ny, double nz) = ComputeRoofNormal(roofNodes);
            Info($"Roof outward normal: ({nx:F3}, {ny:F3}, {nz:F3})");

            int skippedPressure = 0;
            int skippedNode     = 0;
            int mapped          = 0;
            int reportEvery     = Math.Max(1, points.Count / 20);

            for (int i = 0; i < points.Count; i++)
            {
                if (i % reportEvery == 0)
                    Console.Write($"\r  Progress: {i * 100 / points.Count,3}% ({i}/{points.Count})   ");

                var pt = points[i];

                if (Math.Abs(pt.Pressure_kNm2) < MIN_PRESSURE)
                {
                    skippedPressure++;
                    continue;
                }

                int nodeId = FindNearest(grid, pt.Xrobot, pt.Yrobot, pt.Zrobot,
                                         gridSize, SEARCH_R);
                if (nodeId < 0) { skippedNode++; continue; }

                // Force = pressure × area, applied along roof normal
                // Sign convention:
                //   Positive pressure (wind pushing on surface) → force opposes normal
                //     i.e. force = -pressure × area × normal  (pushes inward/downward)
                //   Negative pressure (suction) → force along normal
                //     i.e. force = -pressure × area × normal  (same formula, sign flips)
                // Using: F_vec = -P [kN/m²] × A [m²] × n_hat
                double force_kN = pt.Pressure_kNm2 * cellArea; // positive = push toward surface

                double fx = -force_kN * nx;
                double fy = -force_kN * ny;
                double fz = -force_kN * nz;

                if (nodeForces.TryGetValue(nodeId, out var existing))
                    nodeForces[nodeId] = (existing.Fx + fx,
                                         existing.Fy + fy,
                                         existing.Fz + fz);
                else
                    nodeForces[nodeId] = (fx, fy, fz);

                mapped++;
            }

            Console.WriteLine($"\r  Progress: 100% ({points.Count}/{points.Count})   ");
            OK($"{mapped} points mapped to {nodeForces.Count} nodes");
            if (skippedPressure > 0) Warn($"{skippedPressure} points skipped (|P| < {MIN_PRESSURE} kN/m²)");
            if (skippedNode     > 0) Warn($"{skippedNode} points skipped (no Robot node within {SEARCH_R} m)");

            // ── Step 10 : Write forces to Robot ──────────────────────
            Step(10, "Writing nodal forces to Robot");

            double totalFx = 0, totalFy = 0, totalFz = 0;
            int    loadCount = 0;

            foreach (var kvp in nodeForces)
            {
                double fx  = kvp.Value.Fx;
                double fy  = kvp.Value.Fy;
                double fz  = kvp.Value.Fz;
                double mag = Math.Sqrt(fx*fx + fy*fy + fz*fz);
                if (mag < 0.001) continue;

                int recIdx = simpleCase.Records.New(
                    IRobotLoadRecordType.I_LRT_NODE_FORCE);
                var rec = (RobotLoadRecord)simpleCase.Records.Get(recIdx);

                rec.Objects.FromText(kvp.Key.ToString());
                rec.SetValue((int)IRobotNodeForceRecordValues.I_NFRV_FX, fx);
                rec.SetValue((int)IRobotNodeForceRecordValues.I_NFRV_FY, fy);
                rec.SetValue((int)IRobotNodeForceRecordValues.I_NFRV_FZ, fz);

                totalFx += fx;
                totalFy += fy;
                totalFz += fz;
                loadCount++;
            }

            OK($"{loadCount} nodal loads written");

            // ── Step 11 : Save & report ───────────────────────────────
            project.Save();
            totalTimer.Stop();

            double totalMag = Math.Sqrt(totalFx*totalFx + totalFy*totalFy + totalFz*totalFz);

            Banner("Results summary");
            Console.WriteLine($"  Load case        : #{caseNum}  \"{CASE_NAME}\"");
            Console.WriteLine($"  Nodes loaded     : {loadCount}");
            Console.WriteLine($"  Total Fx         : {totalFx:+0.00;-0.00} kN");
            Console.WriteLine($"  Total Fy         : {totalFy:+0.00;-0.00} kN");
            Console.WriteLine($"  Total Fz         : {totalFz:+0.00;-0.00} kN");
            Console.WriteLine($"  Resultant        : {totalMag:F2} kN");
            Console.WriteLine($"  Elapsed          : {totalTimer.ElapsedMilliseconds} ms");
            Console.WriteLine();
            OK("Project saved. Done.");

            PauseIfInteractive(args);
        }

        // ════════════════════════════════════════════════════════════
        // Read RWIND CSV
        // Format (tab-separated, 1 header line):
        //   Res_Surface_P  Points:0  Points:1  Points:2
        //   [Pa]           X_rwind   Y_rwind   Z_rwind
        // ════════════════════════════════════════════════════════════
        static List<RwindPoint> ReadCsv(string path)
        {
            var result = new List<RwindPoint>();

            if (!File.Exists(path))
            {
                Fatal($"File not found: {path}");
                return result;
            }

            using var reader = new StreamReader(path, System.Text.Encoding.UTF8);

            // Parse header to detect column indices
            string header = reader.ReadLine();
            if (header == null) { Fatal("CSV is empty."); return result; }

            char   sep  = DetectSeparator(header);
            var    cols = header.Split(sep);
            Info($"Separator: '{(sep == '\t' ? "TAB" : sep.ToString())}'  |  Columns: {cols.Length}");

            // Column detection (flexible naming)
            int iP  = FindCol(cols, "res_surface_p", "pressure", "p [pa]", "cp");
            int iX  = FindCol(cols, "points:0", "x [m]", "x");
            int iY  = FindCol(cols, "points:1", "y [m]", "y");
            int iZ  = FindCol(cols, "points:2", "z [m]", "z");

            // Positional fallback (RWIND default order)
            if (iP < 0) { iP = 0; Warn("Pressure col not detected → using col 0"); }
            if (iX < 0) { iX = 1; Warn("X col not detected → using col 1"); }
            if (iY < 0) { iY = 2; Warn("Y col not detected → using col 2"); }
            if (iZ < 0) { iZ = 3; Warn("Z col not detected → using col 3"); }

            Info($"Column mapping: P={iP}  X={iX}  Y={iY}  Z={iZ}");

            int need = new[] { iP, iX, iY, iZ }.Max();
            int line = 1;
            string row;

            while ((row = reader.ReadLine()) != null)
            {
                line++;
                if (string.IsNullOrWhiteSpace(row)) continue;

                var c = row.Split(sep);
                if (c.Length <= need)
                {
                    Warn($"Line {line}: only {c.Length} cols, expected >{need} — skipped");
                    continue;
                }

                try
                {
                    result.Add(new RwindPoint
                    {
                        Pressure_Pa = ParseDouble(c[iP]),
                        Xr          = ParseDouble(c[iX]),
                        Yr          = ParseDouble(c[iY]),
                        Zr          = ParseDouble(c[iZ]),
                    });
                }
                catch (Exception ex)
                {
                    Warn($"Line {line}: parse error — {ex.Message}");
                }
            }

            return result;
        }

        // ════════════════════════════════════════════════════════════
        // Estimate cell area from the sorted unique steps in X and Y
        // of the RWIND grid (original coordinates)
        // ════════════════════════════════════════════════════════════
        static double EstimateCellArea(List<RwindPoint> pts)
        {
            const double tol = 1e-4;

            double dx = MinStep(pts.Select(p => p.Xr).ToList(), tol);
            double dy = MinStep(pts.Select(p => p.Yr).ToList(), tol);

            // If one axis is constant (e.g. flat vertical surface), try Z
            if (dx < tol) dx = MinStep(pts.Select(p => p.Zr).ToList(), tol);
            if (dy < tol) dy = MinStep(pts.Select(p => p.Zr).ToList(), tol);

            double area = dx * dy;

            if (area < 0.001 || area > 25.0)
            {
                Warn($"Computed area {area:F4} m² out of range " +
                     $"(dx={dx:F3} dy={dy:F3}) → using 0.25 m²");
                return 0.25;
            }

            return area;
        }

        static double MinStep(List<double> vals, double tol)
        {
            var sorted = vals.Distinct().OrderBy(v => v).ToList();
            double min = double.MaxValue;
            for (int i = 1; i < sorted.Count; i++)
            {
                double s = sorted[i] - sorted[i - 1];
                if (s > tol && s < min) min = s;
            }
            return min == double.MaxValue ? 1.0 : min;
        }

        // ════════════════════════════════════════════════════════════
        // Compute roof outward normal from actual node geometry.
        // For an inclined monopitch roof:
        //   two edge nodes define the slope; cross-product → normal.
        // Falls back to (0, 0, 1) if geometry is ambiguous.
        // ════════════════════════════════════════════════════════════
        static (double nx, double ny, double nz) ComputeRoofNormal(List<RoboNode> nodes)
        {
            if (nodes.Count < 3) return (0, 0, 1);

            // Take 3 well-separated nodes to define the roof plane
            var sorted = nodes.OrderBy(n => n.X).ThenBy(n => n.Y).ToList();
            var A = sorted[0];
            var B = sorted[sorted.Count / 2];
            var C = sorted[sorted.Count - 1];

            double ax = B.X - A.X, ay = B.Y - A.Y, az = B.Z - A.Z;
            double bx = C.X - A.X, by = C.Y - A.Y, bz = C.Z - A.Z;

            // Cross product A→B × A→C
            double cx = ay * bz - az * by;
            double cy = az * bx - ax * bz;
            double cz = ax * by - ay * bx;

            double len = Math.Sqrt(cx*cx + cy*cy + cz*cz);
            if (len < 1e-6) return (0, 0, 1);

            cx /= len; cy /= len; cz /= len;

            // Ensure normal points upward (nz > 0)
            if (cz < 0) { cx = -cx; cy = -cy; cz = -cz; }

            return (cx, cy, cz);
        }

        // ════════════════════════════════════════════════════════════
        // Auto-detect grid size from median nearest-neighbour distance
        // ════════════════════════════════════════════════════════════
        static double EstimateGridSize(List<RoboNode> nodes)
        {
            int sample = Math.Min(nodes.Count, 200);
            var dists = new List<double>(sample);

            for (int i = 0; i < sample; i++)
            {
                double best = double.MaxValue;
                for (int j = 0; j < sample; j++)
                {
                    if (i == j) continue;
                    double d = Dist3(nodes[i].X, nodes[i].Y, nodes[i].Z,
                                     nodes[j].X, nodes[j].Y, nodes[j].Z);
                    if (d < best) best = d;
                }
                if (best < double.MaxValue) dists.Add(best);
            }

            if (dists.Count == 0) return 1.0;
            dists.Sort();
            return Math.Max(0.1, dists[dists.Count / 2] * 1.5);
        }

        // ════════════════════════════════════════════════════════════
        // Spatial grid: stores full node data, resolves collisions
        // ════════════════════════════════════════════════════════════
        static Dictionary<(int,int,int), RoboNode> BuildGrid(
            List<RoboNode> nodes, double gs)
        {
            var grid = new Dictionary<(int,int,int), RoboNode>();

            foreach (var n in nodes)
            {
                int gx = (int)Math.Round(n.X / gs);
                int gy = (int)Math.Round(n.Y / gs);
                int gz = (int)Math.Round(n.Z / gs);
                var key = (gx, gy, gz);

                if (grid.TryGetValue(key, out var ex))
                {
                    // Keep the node closest to the cell centre
                    double ccx = gx * gs, ccy = gy * gs, ccz = gz * gs;
                    if (Dist3(n.X, n.Y, n.Z, ccx, ccy, ccz) <
                        Dist3(ex.X, ex.Y, ex.Z, ccx, ccy, ccz))
                        grid[key] = n;
                }
                else grid[key] = n;
            }
            return grid;
        }

        static int FindNearest(Dictionary<(int,int,int), RoboNode> grid,
                               double x, double y, double z,
                               double gs, double maxR)
        {
            int gx = (int)Math.Round(x / gs);
            int gy = (int)Math.Round(y / gs);
            int gz = (int)Math.Round(z / gs);

            int    best = -1;
            double bestD = double.MaxValue;

            for (int dx = -2; dx <= 2; dx++)
            for (int dy = -2; dy <= 2; dy++)
            for (int dz = -2; dz <= 2; dz++)
            {
                if (!grid.TryGetValue((gx+dx, gy+dy, gz+dz), out var n)) continue;
                double d = Dist3(n.X, n.Y, n.Z, x, y, z);
                if (d < bestD) { bestD = d; best = n.Id; }
            }

            return bestD <= maxR ? best : -1;
        }

        // ════════════════════════════════════════════════════════════
        // Helpers
        // ════════════════════════════════════════════════════════════
        static double Dist3(double x1, double y1, double z1,
                            double x2, double y2, double z2)
            => Math.Sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2));

        static char DetectSeparator(string h)
        {
            int t = h.Count(c => c == '\t');
            int s = h.Count(c => c == ';');
            int cm = h.Count(c => c == ',');
            if (t >= s && t >= cm) return '\t';
            return s >= cm ? ';' : ',';
        }

        static int FindCol(string[] cols, params string[] keys)
        {
            for (int i = 0; i < cols.Length; i++)
            {
                string cl = cols[i].Trim().ToLowerInvariant().Trim('"');
                foreach (var k in keys)
                    if (cl.Contains(k)) return i;
            }
            return -1;
        }

        static double ParseDouble(string v)
        {
            v = v.Trim().Trim('"');
            if (double.TryParse(v, NumberStyles.Any, CultureInfo.InvariantCulture, out double r)) return r;
            if (double.TryParse(v, NumberStyles.Any, new CultureInfo("fr-FR"), out double r2)) return r2;
            string fix = v.Replace(',', '.');
            if (double.TryParse(fix, NumberStyles.Any, CultureInfo.InvariantCulture, out double r3)) return r3;
            throw new FormatException($"Cannot parse '{v}'");
        }

        static void PauseIfInteractive(string[] args)
        {
            if (!Array.Exists(args, a =>
                a.Equals("--no-wait", StringComparison.OrdinalIgnoreCase)))
            {
                Console.WriteLine("\nPress any key to exit...");
                Console.ReadKey();
            }
        }

        // ── Console helpers ───────────────────────────────────────────
        static void Banner(string t) {
            Console.WriteLine();
            Console.WriteLine($"╔══ {t} ══");
            Console.WriteLine();
        }
        static void Step(int n, string t) =>
            Console.WriteLine($"\n── Step {n}: {t}");
        static void OK  (string m) => Console.WriteLine($"  ✓  {m}");
        static void Info(string m) => Console.WriteLine($"  ·  {m}");
        static void Warn(string m) => Console.WriteLine($"  ⚠  {m}");
        static void Fatal(string m) {
            Console.WriteLine($"  ✗  FATAL: {m}");
            Console.WriteLine("\nPress any key to exit...");
            Console.ReadKey();
        }
    }
}
