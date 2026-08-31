package touch

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/vision"
	"go.viam.com/rdk/spatialmath"
	viz "go.viam.com/rdk/vision"

	"github.com/erh/vmodutils"
	"github.com/erh/vmodutils/pcclean"
)

var MergeAllObjectsModel = vmodutils.NamespaceFamily.WithModel("merge-all-objects-pointclouds")

func init() {
	resource.RegisterComponent(
		camera.API,
		MergeAllObjectsModel,
		resource.Registration[camera.Camera, *MergeAllObjectsConfig]{
			Constructor: newMergeAllObjects,
		})
}

// VisionServiceSource is one GetObjectPointClouds dependency for the merge camera.
// MinObjects is the minimum number of label-matching, non-empty objects required
// from this service on each NextPointCloud call. 0 means the source is optional
// (errors / empty results are skipped, matching legacy string-list behavior).
type VisionServiceSource struct {
	Name       string `json:"name"`
	MinObjects int    `json:"min_objects"`
}

// VisionServiceSourceList unmarshals either a legacy string list
// (["left","right"] → MinObjects 0) or a list of source objects
// ([{"name":"left","min_objects":1}, ...]).
type VisionServiceSourceList []VisionServiceSource

func (l *VisionServiceSourceList) UnmarshalJSON(data []byte) error {
	if len(data) == 0 || string(data) == "null" {
		*l = nil
		return nil
	}

	var names []string
	if err := json.Unmarshal(data, &names); err == nil {
		out := make(VisionServiceSourceList, len(names))
		for i, n := range names {
			out[i] = VisionServiceSource{Name: n, MinObjects: 0}
		}
		*l = out
		return nil
	}

	type rawSource struct {
		Name       string `json:"name"`
		MinObjects *int   `json:"min_objects"`
	}
	var raw []rawSource
	if err := json.Unmarshal(data, &raw); err != nil {
		return fmt.Errorf("vision_services must be a list of strings or objects with name/min_objects: %w", err)
	}
	out := make(VisionServiceSourceList, len(raw))
	for i, r := range raw {
		if r.Name == "" {
			return fmt.Errorf("vision_services[%d]: name is required", i)
		}
		minObjects := 0
		if r.MinObjects != nil {
			minObjects = *r.MinObjects
		}
		if minObjects < 0 {
			return fmt.Errorf("vision_services[%d]: min_objects must be >= 0", i)
		}
		out[i] = VisionServiceSource{Name: r.Name, MinObjects: minObjects}
	}
	*l = out
	return nil
}

func (l VisionServiceSourceList) Names() []string {
	names := make([]string, len(l))
	for i, s := range l {
		names[i] = s.Name
	}
	return names
}

type MergeAllObjectsConfig struct {
	VisionServices VisionServiceSourceList `json:"vision_services"`
	Label          string                  `json:"label"`

	// MinWorldZMM, when set (UseMinWorldZ), drops points with world Z below this
	// value before cleaning — strips table bleed under glass masks.
	MinWorldZMM  float64 `json:"min_world_z_mm,omitempty"`
	UseMinWorldZ bool    `json:"use_min_world_z,omitempty"`

	// BalanceCenters shifts the cleaned cloud so its XY AABB center is the mean of
	// per-source object centers when ≥2 sources contribute and |ΔY| ≤ AgreeDeltaYMM.
	// Default off — cup-eval showed no XY improvement vs merged baseline.
	BalanceCenters *bool   `json:"balance_centers,omitempty"`
	AgreeDeltaYMM  float64 `json:"agree_delta_y_mm,omitempty"`

	// MinSources is the minimum number of vision sources that must contribute at
	// least one matching object. Softer than setting min_objects:1 on every source
	// (one flaky camera no longer fails the whole merge). 0 disables this check.
	MinSources int `json:"min_sources,omitempty"`

	pcclean.Config
}

func (c *MergeAllObjectsConfig) balanceCentersEnabled() bool {
	if c.BalanceCenters == nil {
		return false
	}
	return *c.BalanceCenters
}

func (c *MergeAllObjectsConfig) agreeDeltaYMM() float64 {
	if c.AgreeDeltaYMM > 0 {
		return c.AgreeDeltaYMM
	}
	return 40
}

func (c *MergeAllObjectsConfig) Validate(path string) ([]string, []string, error) {
	if len(c.VisionServices) == 0 {
		return nil, nil, fmt.Errorf("need at least one vision service")
	}
	for i, s := range c.VisionServices {
		if s.Name == "" {
			return nil, nil, fmt.Errorf("vision_services[%d]: name is required", i)
		}
		if s.MinObjects < 0 {
			return nil, nil, fmt.Errorf("vision_services[%d]: min_objects must be >= 0", i)
		}
	}
	if c.MinSources < 0 {
		return nil, nil, fmt.Errorf("min_sources must be >= 0")
	}
	if c.MinSources > len(c.VisionServices) {
		return nil, nil, fmt.Errorf("min_sources (%d) exceeds vision_services length (%d)", c.MinSources, len(c.VisionServices))
	}
	return c.VisionServices.Names(), nil, nil
}

func newMergeAllObjects(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (camera.Camera, error) {
	newConf, err := resource.NativeConfig[*MergeAllObjectsConfig](config)
	if err != nil {
		return nil, err
	}
	pcclean.FillDefaults(&newConf.Config)

	cc := &MergeAllObjectsCamera{
		name:    config.ResourceName(),
		cfg:     newConf,
		sources: make([]mergeAllObjectsSource, 0, len(newConf.VisionServices)),
		logger:  logger,
	}

	for _, src := range newConf.VisionServices {
		s, err := vision.FromProvider(deps, src.Name)
		if err != nil {
			return nil, err
		}
		cc.sources = append(cc.sources, mergeAllObjectsSource{
			svc:        s,
			minObjects: src.MinObjects,
		})
	}

	return cc, nil
}

type mergeAllObjectsSource struct {
	svc        vision.Service
	minObjects int
}

type MergeAllObjectsCamera struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name    resource.Name
	cfg     *MergeAllObjectsConfig
	logger  logging.Logger
	sources []mergeAllObjectsSource

	mu              sync.Mutex
	balanceOverride *bool
	minSourcesOverride *int
	enabledMask     []bool // nil = all enabled; parallel to sources
}

func (opc *MergeAllObjectsCamera) Name() resource.Name {
	return opc.name
}

func (opc *MergeAllObjectsCamera) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (opc *MergeAllObjectsCamera) Images(ctx context.Context, filterSourceNames []string, extra map[string]interface{}) ([]camera.NamedImage, resource.ResponseMetadata, error) {
	pc, err := opc.NextPointCloud(ctx, extra)
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	img := PCToImage(pc)

	ni, err := camera.NamedImageFromImage(img, "merged-objects", "image/png", data.Annotations{})
	if err != nil {
		return nil, resource.ResponseMetadata{}, err
	}
	return []camera.NamedImage{ni}, resource.ResponseMetadata{CapturedAt: time.Now()}, nil
}

func (opc *MergeAllObjectsCamera) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if cmd["status"] == true {
		opc.mu.Lock()
		defer opc.mu.Unlock()
		enabled := make([]string, 0, len(opc.sources))
		for i, src := range opc.sources {
			if opc.sourceEnabledLocked(i) {
				enabled = append(enabled, src.svc.Name().ShortName())
			}
		}
		bal := opc.cfg.balanceCentersEnabled()
		if opc.balanceOverride != nil {
			bal = *opc.balanceOverride
		}
		minSrc := opc.cfg.MinSources
		if opc.minSourcesOverride != nil {
			minSrc = *opc.minSourcesOverride
		}
		return map[string]interface{}{
			"balance_centers":   bal,
			"min_sources":       minSrc,
			"enabled_sources":   enabled,
			"all_sources":       opc.cfg.VisionServices.Names(),
			"params_overridden": opc.balanceOverride != nil || opc.enabledMask != nil || opc.minSourcesOverride != nil,
		}, nil
	}

	if raw, ok := cmd["set_params"]; ok {
		m, ok := raw.(map[string]interface{})
		if !ok {
			return nil, fmt.Errorf("set_params must be a map")
		}
		opc.mu.Lock()
		defer opc.mu.Unlock()
		if v, ok := m["balance_centers"].(bool); ok {
			b := v
			opc.balanceOverride = &b
		}
		if v, ok := asInt(m["min_sources"]); ok {
			opc.minSourcesOverride = &v
		}
		if rawSources, ok := m["enabled_sources"]; ok {
			names, err := stringList(rawSources)
			if err != nil {
				return nil, err
			}
			mask := make([]bool, len(opc.sources))
			if len(names) == 0 {
				for i := range mask {
					mask[i] = true
				}
			} else {
				want := map[string]bool{}
				for _, n := range names {
					want[n] = true
				}
				for i, src := range opc.sources {
					short := src.svc.Name().ShortName()
					full := src.svc.Name().String()
					cfgName := ""
					if i < len(opc.cfg.VisionServices) {
						cfgName = opc.cfg.VisionServices[i].Name
					}
					mask[i] = want[short] || want[full] || want[cfgName]
				}
			}
			opc.enabledMask = mask
		}
		return opc.statusLocked(), nil
	}

	if cmd["reset_params"] == true {
		opc.mu.Lock()
		defer opc.mu.Unlock()
		opc.balanceOverride = nil
		opc.minSourcesOverride = nil
		opc.enabledMask = nil
		return opc.statusLocked(), nil
	}

	return nil, nil
}

func asInt(v interface{}) (int, bool) {
	switch x := v.(type) {
	case float64:
		return int(x), true
	case int:
		return x, true
	case int64:
		return int(x), true
	default:
		return 0, false
	}
}

func stringList(raw interface{}) ([]string, error) {
	switch v := raw.(type) {
	case []string:
		return v, nil
	case []interface{}:
		out := make([]string, 0, len(v))
		for i, x := range v {
			s, ok := x.(string)
			if !ok {
				return nil, fmt.Errorf("enabled_sources[%d] must be string", i)
			}
			out = append(out, s)
		}
		return out, nil
	default:
		return nil, fmt.Errorf("enabled_sources must be a string list")
	}
}

func (opc *MergeAllObjectsCamera) sourceEnabledLocked(i int) bool {
	if opc.enabledMask == nil {
		return true
	}
	if i < 0 || i >= len(opc.enabledMask) {
		return false
	}
	return opc.enabledMask[i]
}

func (opc *MergeAllObjectsCamera) statusLocked() map[string]interface{} {
	enabled := make([]string, 0, len(opc.sources))
	for i, src := range opc.sources {
		if opc.sourceEnabledLocked(i) {
			enabled = append(enabled, src.svc.Name().ShortName())
		}
	}
	bal := opc.cfg.balanceCentersEnabled()
	if opc.balanceOverride != nil {
		bal = *opc.balanceOverride
	}
	minSrc := opc.cfg.MinSources
	if opc.minSourcesOverride != nil {
		minSrc = *opc.minSourcesOverride
	}
	return map[string]interface{}{
		"balance_centers":   bal,
		"min_sources":       minSrc,
		"enabled_sources":   enabled,
		"all_sources":       opc.cfg.VisionServices.Names(),
		"params_overridden": opc.balanceOverride != nil || opc.enabledMask != nil || opc.minSourcesOverride != nil,
	}
}

type sourceFetchResult struct {
	idx     int
	objects []*viz.Object
	err     error
}

func (opc *MergeAllObjectsCamera) NextPointCloud(ctx context.Context, extra map[string]interface{}) (pointcloud.PointCloud, error) {
	opc.mu.Lock()
	balanceOn := opc.cfg.balanceCentersEnabled()
	if opc.balanceOverride != nil {
		balanceOn = *opc.balanceOverride
	}
	minSources := opc.cfg.MinSources
	if opc.minSourcesOverride != nil {
		minSources = *opc.minSourcesOverride
	}
	enabled := make([]bool, len(opc.sources))
	for i := range opc.sources {
		enabled[i] = opc.sourceEnabledLocked(i)
	}
	agreeDelta := opc.cfg.agreeDeltaYMM()
	opc.mu.Unlock()

	results := make([]sourceFetchResult, len(opc.sources))
	var wg sync.WaitGroup
	for i, src := range opc.sources {
		if !enabled[i] {
			continue
		}
		wg.Add(1)
		go func(i int, src mergeAllObjectsSource) {
			defer wg.Done()
			objects, err := src.svc.GetObjectPointClouds(ctx, "", extra)
			results[i] = sourceFetchResult{idx: i, objects: objects, err: err}
		}(i, src)
	}
	wg.Wait()

	inputs := []pointcloud.PointCloud{}
	totalSize := 0
	sourceCenters := []r3.Vector{}
	sourcesUsed := 0

	for i, src := range opc.sources {
		if !enabled[i] {
			continue
		}
		res := results[i]
		if res.err != nil {
			if src.minObjects > 0 {
				return nil, fmt.Errorf("required vision service %s failed GetObjectPointClouds: %w", src.svc.Name(), res.err)
			}
			opc.logger.Warnf("error getting object point clouds from %s: %v", src.svc.Name(), res.err)
			continue
		}

		count := 0
		var bestCenter r3.Vector
		bestSize := 0
		for _, obj := range res.objects {
			if opc.cfg.Label != "" && obj.Geometry != nil {
				if obj.Geometry.Label() != opc.cfg.Label {
					continue
				}
			}
			if obj.PointCloud == nil || obj.Size() == 0 {
				continue
			}
			count++
			totalSize += obj.Size()
			inputs = append(inputs, obj.PointCloud)
			if obj.Size() > bestSize {
				bestSize = obj.Size()
				md := obj.MetaData()
				bestCenter = md.Center()
			}
		}

		if count < src.minObjects {
			return nil, fmt.Errorf(
				"vision service %s: got %d matching object(s), need at least %d",
				src.svc.Name(), count, src.minObjects,
			)
		}
		if count > 0 {
			sourceCenters = append(sourceCenters, bestCenter)
			sourcesUsed++
		}
	}

	if minSources > 0 && sourcesUsed < minSources {
		return nil, fmt.Errorf(
			"got objects from %d vision source(s), need at least min_sources=%d",
			sourcesUsed, minSources,
		)
	}

	big := pointcloud.NewBasicPointCloud(totalSize)
	for _, pc := range inputs {
		if err := pointcloud.ApplyOffset(pc, nil, big); err != nil {
			return nil, err
		}
	}

	if opc.cfg.UseMinWorldZ {
		filtered, nDropped := filterMinWorldZ(big, opc.cfg.MinWorldZMM)
		opc.logger.Debugf("merge-all-objects: min_world_z_mm=%.1f dropped %d/%d points",
			opc.cfg.MinWorldZMM, nDropped, big.Size())
		big = filtered
	}

	cleaned, err := pcclean.Clean(big, &opc.cfg.Config)
	if err != nil {
		return nil, err
	}

	if balanceOn {
		cleaned = balanceMergedCenter(cleaned, sourceCenters, agreeDelta, opc.logger)
	}
	return cleaned, nil
}

// balanceMergedCenter shifts cleaned so its XY AABB center matches the mean of
// source centers when |ΔY| between the first two sources is within agreeDeltaY.
func balanceMergedCenter(cleaned pointcloud.PointCloud, sourceCenters []r3.Vector, agreeDeltaY float64, logger logging.Logger) pointcloud.PointCloud {
	if cleaned == nil || cleaned.Size() == 0 || len(sourceCenters) < 2 {
		return cleaned
	}
	c0, c1 := sourceCenters[0], sourceCenters[1]
	dy := c0.Y - c1.Y
	if dy < 0 {
		dy = -dy
	}
	if dy > agreeDeltaY {
		if logger != nil {
			logger.Debugf("merge-all-objects: skip balance centers ΔY=%.1f > %.1f", dy, agreeDeltaY)
		}
		return cleaned
	}
	mean := r3.Vector{
		X: (c0.X + c1.X) / 2,
		Y: (c0.Y + c1.Y) / 2,
	}
	md := cleaned.MetaData()
	cur := md.Center()
	offset := r3.Vector{X: mean.X - cur.X, Y: mean.Y - cur.Y}
	if offset.X == 0 && offset.Y == 0 {
		return cleaned
	}
	out := pointcloud.NewBasicPointCloud(cleaned.Size())
	pose := spatialmath.NewPoseFromPoint(offset)
	if err := pointcloud.ApplyOffset(cleaned, pose, out); err != nil {
		if logger != nil {
			logger.Warnf("merge-all-objects: balance centers ApplyOffset failed: %v", err)
		}
		return cleaned
	}
	if logger != nil {
		logger.Debugf("merge-all-objects: balanced center XY by (%.1f, %.1f) ΔY=%.1f", offset.X, offset.Y, dy)
	}
	return out
}

// filterMinWorldZ keeps points with Z >= minZ. Returns filtered cloud and drop count.
func filterMinWorldZ(in pointcloud.PointCloud, minZ float64) (pointcloud.PointCloud, int) {
	if in == nil || in.Size() == 0 {
		return in, 0
	}
	out := pointcloud.NewBasicEmpty()
	dropped := 0
	in.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if p.Z < minZ {
			dropped++
			return true
		}
		_ = out.Set(p, d)
		return true
	})
	return out, dropped
}

func (opc *MergeAllObjectsCamera) Properties(ctx context.Context) (camera.Properties, error) {
	return camera.Properties{
		SupportsPCD: true,
	}, nil
}

func (opc *MergeAllObjectsCamera) Geometries(ctx context.Context, _ map[string]interface{}) ([]spatialmath.Geometry, error) {
	return nil, nil
}
