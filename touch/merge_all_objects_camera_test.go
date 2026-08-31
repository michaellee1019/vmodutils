package touch

import (
	"context"
	"encoding/json"
	"errors"
	"testing"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/testutils/inject"
	viz "go.viam.com/rdk/vision"
	"go.viam.com/test"

	"github.com/erh/vmodutils/pcclean"
)

func TestVisionServiceSourceListUnmarshal(t *testing.T) {
	t.Run("legacy string list is optional", func(t *testing.T) {
		var list VisionServiceSourceList
		err := json.Unmarshal([]byte(`["left","right"]`), &list)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, list, test.ShouldResemble, VisionServiceSourceList{
			{Name: "left", MinObjects: 0},
			{Name: "right", MinObjects: 0},
		})
	})

	t.Run("object list with min_objects", func(t *testing.T) {
		var list VisionServiceSourceList
		err := json.Unmarshal([]byte(`[
			{"name":"left","min_objects":1},
			{"name":"right","min_objects":0}
		]`), &list)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, list, test.ShouldResemble, VisionServiceSourceList{
			{Name: "left", MinObjects: 1},
			{Name: "right", MinObjects: 0},
		})
	})

	t.Run("object list omits min_objects as zero", func(t *testing.T) {
		var list VisionServiceSourceList
		err := json.Unmarshal([]byte(`[{"name":"only"}]`), &list)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, list[0].MinObjects, test.ShouldEqual, 0)
	})

	t.Run("rejects missing name", func(t *testing.T) {
		var list VisionServiceSourceList
		err := json.Unmarshal([]byte(`[{"min_objects":1}]`), &list)
		test.That(t, err, test.ShouldNotBeNil)
	})
}

func TestMergeAllObjectsConfigValidate(t *testing.T) {
	cfg := &MergeAllObjectsConfig{
		VisionServices: VisionServiceSourceList{
			{Name: "left", MinObjects: 1},
			{Name: "right", MinObjects: 1},
		},
	}
	deps, opt, err := cfg.Validate("components.0")
	test.That(t, err, test.ShouldBeNil)
	test.That(t, opt, test.ShouldBeNil)
	test.That(t, deps, test.ShouldResemble, []string{"left", "right"})
}

func TestFilterMinWorldZ(t *testing.T) {
	pc := pointcloud.NewBasicEmpty()
	test.That(t, pc.Set(r3.Vector{X: 0, Y: 0, Z: -5}, pointcloud.NewBasicData()), test.ShouldBeNil)
	test.That(t, pc.Set(r3.Vector{X: 0, Y: 0, Z: 10}, pointcloud.NewBasicData()), test.ShouldBeNil)
	test.That(t, pc.Set(r3.Vector{X: 0, Y: 0, Z: 50}, pointcloud.NewBasicData()), test.ShouldBeNil)
	out, dropped := filterMinWorldZ(pc, 0)
	test.That(t, dropped, test.ShouldEqual, 1)
	test.That(t, out.Size(), test.ShouldEqual, 2)
}

func TestBalanceMergedCenter(t *testing.T) {
	logger := logging.NewTestLogger(t)
	// Dense cloud biased to +Y; source centers agree near Y=0.
	pc := pointcloud.NewBasicEmpty()
	for y := 20.0; y <= 40; y += 5 {
		test.That(t, pc.Set(r3.Vector{X: 100, Y: y, Z: 50}, pointcloud.NewBasicData()), test.ShouldBeNil)
	}
	out := balanceMergedCenter(pc, []r3.Vector{
		{X: 100, Y: -5, Z: 50},
		{X: 100, Y: 5, Z: 50},
	}, 40, logger)
	md := out.MetaData()
	c := md.Center()
	test.That(t, c.X, test.ShouldAlmostEqual, 100, 0.1)
	test.That(t, c.Y, test.ShouldAlmostEqual, 0, 0.1)

	// ΔY too large: no shift
	out2 := balanceMergedCenter(pc, []r3.Vector{
		{X: 100, Y: -50, Z: 50},
		{X: 100, Y: 50, Z: 50},
	}, 40, logger)
	md2 := out2.MetaData()
	mdPC := pc.MetaData()
	test.That(t, md2.Center().Y, test.ShouldAlmostEqual, mdPC.Center().Y, 0.1)
}

func TestMergeAllObjectsNextPointCloudMinObjects(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()

	makePC := func(offset float64) pointcloud.PointCloud {
		pc := pointcloud.NewBasicEmpty()
		test.That(t, pc.Set(r3.Vector{X: 1 + offset, Y: 2, Z: 3}, pointcloud.NewBasicData()), test.ShouldBeNil)
		test.That(t, pc.Set(r3.Vector{X: 4 + offset, Y: 5, Z: 6}, pointcloud.NewBasicData()), test.ShouldBeNil)
		return pc
	}

	makeObj := func(offset float64) *viz.Object {
		obj, err := viz.NewObject(makePC(offset))
		test.That(t, err, test.ShouldBeNil)
		return obj
	}

	left := inject.NewVisionService("left")
	right := inject.NewVisionService("right")
	deps := resource.Dependencies{
		left.Name():  left,
		right.Name(): right,
	}

	build := func(sources VisionServiceSourceList) *MergeAllObjectsCamera {
		cfg := &MergeAllObjectsConfig{
			VisionServices: sources,
			Config:         pcclean.Config{Disable: true},
		}
		cam, err := newMergeAllObjects(ctx, deps, resource.Config{
			Name:                "merged",
			ConvertedAttributes: cfg,
		}, logger)
		test.That(t, err, test.ShouldBeNil)
		return cam.(*MergeAllObjectsCamera)
	}

	t.Run("required sources both present", func(t *testing.T) {
		left.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(0)}, nil
		}
		right.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(10)}, nil
		}
		cam := build(VisionServiceSourceList{
			{Name: "left", MinObjects: 1},
			{Name: "right", MinObjects: 1},
		})
		pc, err := cam.NextPointCloud(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldEqual, 4)
	})

	t.Run("required source missing objects fails closed", func(t *testing.T) {
		left.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(0)}, nil
		}
		right.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{}, nil
		}
		cam := build(VisionServiceSourceList{
			{Name: "left", MinObjects: 1},
			{Name: "right", MinObjects: 1},
		})
		_, err := cam.NextPointCloud(ctx, nil)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "right")
		test.That(t, err.Error(), test.ShouldContainSubstring, "need at least 1")
	})

	t.Run("required source error fails closed", func(t *testing.T) {
		left.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(0)}, nil
		}
		right.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return nil, errors.New("boom")
		}
		cam := build(VisionServiceSourceList{
			{Name: "left", MinObjects: 1},
			{Name: "right", MinObjects: 1},
		})
		_, err := cam.NextPointCloud(ctx, nil)
		test.That(t, err, test.ShouldNotBeNil)
		test.That(t, err.Error(), test.ShouldContainSubstring, "required vision service")
		test.That(t, err.Error(), test.ShouldContainSubstring, "right")
	})

	t.Run("legacy optional source still soft-fails", func(t *testing.T) {
		left.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return []*viz.Object{makeObj(0)}, nil
		}
		right.GetObjectPointCloudsFunc = func(ctx context.Context, cameraName string, extra map[string]interface{}) ([]*viz.Object, error) {
			return nil, errors.New("boom")
		}
		cam := build(VisionServiceSourceList{
			{Name: "left", MinObjects: 0},
			{Name: "right", MinObjects: 0},
		})
		pc, err := cam.NextPointCloud(ctx, nil)
		test.That(t, err, test.ShouldBeNil)
		test.That(t, pc.Size(), test.ShouldEqual, 2)
	})
}

func TestMergeSetParamsDoCommand(t *testing.T) {
	logger := logging.NewTestLogger(t)
	ctx := context.Background()
	left := inject.NewVisionService("sam2-segmenter-left")
	right := inject.NewVisionService("sam2-segmenter-right")
	deps := resource.Dependencies{
		left.Name():  left,
		right.Name(): right,
	}
	cfg := &MergeAllObjectsConfig{
		VisionServices: VisionServiceSourceList{
			{Name: "sam2-segmenter-left"},
			{Name: "sam2-segmenter-right"},
		},
		Config: pcclean.Config{Disable: true},
	}
	camRes, err := newMergeAllObjects(ctx, deps, resource.Config{
		Name:                "merged",
		ConvertedAttributes: cfg,
	}, logger)
	test.That(t, err, test.ShouldBeNil)
	opc := camRes.(*MergeAllObjectsCamera)

	st, err := opc.DoCommand(ctx, map[string]interface{}{
		"set_params": map[string]interface{}{
			"balance_centers": false,
			"min_sources":     1.0,
			"enabled_sources": []interface{}{"sam2-segmenter-left"},
		},
	})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["balance_centers"], test.ShouldEqual, false)
	test.That(t, st["min_sources"], test.ShouldEqual, 1)
	test.That(t, st["params_overridden"], test.ShouldEqual, true)
	enabled := st["enabled_sources"].([]string)
	test.That(t, enabled, test.ShouldResemble, []string{"sam2-segmenter-left"})

	st, err = opc.DoCommand(ctx, map[string]interface{}{"reset_params": true})
	test.That(t, err, test.ShouldBeNil)
	test.That(t, st["params_overridden"], test.ShouldEqual, false)
}
