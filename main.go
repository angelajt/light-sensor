package main

import (
	"github.com/deadsy/sdfx/render"
	"github.com/deadsy/sdfx/sdf"
	. "github.com/stevegt/goadapt"
)

const (
	// common
	in    = 25.4
	scale = 1.0 // PETG no shrinkage

	// E450 brake rotor hub
	hubId    = 100.0
	hubOd    = 110.0
	hubDepth = 20.0

	wall    = 3.0
	spacing = 3.0
)

// adafruit BNO055 IMU
var imu = &Board{
	X:       1.05 * in,
	Y:       0.8 * in,
	Z:       15.0,
	CtrsX:   .85 * in,
	CtrsY:   .6 * in,
	PostDia: 2.0, // hole dia 2.54
}

// feather
var feather = &Board{
	X:       2.0 * in,
	Y:       0.9 * in,
	Z:       15.0,
	CtrsX:   1.8 * in,
	CtrsY:   .7 * in,
	PostDia: 2.0, // hole dia 2.54
}

// adafruit battery 328
var battery = &Battery{
	X:       62.5,
	Y:       50.5,
	Z:       8.1,
	CornerW: wall + 10,
}

var body = &Body{
	Od1: hubId - 5,
	Od2: hubId,
}

type Board struct {
	X         float64
	Y         float64
	Z         float64
	CtrsX     float64
	CtrsY     float64
	PostDia   float64
	PostZ     float64
	ShoulderZ float64
}

func (b *Board) posts() (res sdf.SDF3) {
	var posts []sdf.SDF3
	for ix := 0; ix < 2; ix++ {
		for iy := 0; iy < 2; iy++ {
			post, err := sdf.Cylinder3D(b.PostZ, b.PostDia/2.0, 0.0)
			Ck(err)
			post = sdf.Transform3D(post, sdf.Translate3d(sdf.V3{0, 0, b.PostZ / 2.0}))
			shoulder, err := sdf.Cylinder3D(b.ShoulderZ, b.PostDia, 0.0)
			Ck(err)
			shoulder = sdf.Transform3D(shoulder, sdf.Translate3d(sdf.V3{0, 0, b.ShoulderZ / 2.0}))
			post = sdf.Union3D(post, shoulder)
			x := float64(ix)*b.CtrsX - b.CtrsX/2.0
			y := float64(iy)*b.CtrsY - b.CtrsY/2.0
			post = sdf.Transform3D(post, sdf.Translate3d(sdf.V3{x, y, 0}))
			posts = append(posts, post)
		}
	}
	return sdf.Union3D(posts...)
}

type Battery struct {
	X       float64
	Y       float64
	Z       float64
	CornerW float64
	CornerZ float64
}

func (b *Battery) corners() (res sdf.SDF3) {
	res, err := sdf.Box3D(sdf.V3{b.X + wall*2, b.Y + wall*2, b.CornerZ}, 1)
	Ck(err)
	hCut, err := sdf.Box3D(sdf.V3{b.X - b.CornerW*2 + wall*2, b.Y * 2, b.CornerZ}, 0)
	Ck(err)
	vCut, err := sdf.Box3D(sdf.V3{b.X * 2, b.Y - b.CornerW*2 + wall*2, b.CornerZ}, 0)
	Ck(err)
	res = sdf.Difference3D(res, hCut)
	res = sdf.Difference3D(res, vCut)
	// res = sdf.Difference3D(res, b.shape())
	return
}

func (b *Battery) shape() (res sdf.SDF3) {
	res, err := sdf.Box3D(sdf.V3{b.X, b.Y, b.Z}, 0)
	Ck(err)
	return
}

type Body struct {
	Od1 float64
	Od2 float64
	Z   float64
	Id  float64
}

func (b *Body) shape() (res sdf.SDF3) {
	b.Id = b.Od1 - wall*2
	lipOd := b.Od2 + 4
	lipZ := hubDepth - wall

	depth1 := hubDepth / 2.0
	depth2 := body.Z - depth1

	circle1, err := sdf.Circle2D(b.Od1 / 2.0)
	Ck(err)

	circle2, err := sdf.Circle2D(b.Od2 / 2.0)
	Ck(err)

	res, err = sdf.Loft3D(circle1, circle2, depth1, 0)
	Ck(err)
	res = sdf.Transform3D(res, sdf.Translate3d(sdf.V3{0, 0, depth1 / 2.0}))

	cyl2, err := sdf.Cylinder3D(depth2, b.Od2/2.0, 0)
	Ck(err)
	cyl2 = sdf.Transform3D(cyl2, sdf.Translate3d(sdf.V3{0, 0, depth1 + depth2/2.0}))
	res = sdf.Union3D(res, cyl2)

	lip, err := sdf.Cylinder3D(wall, lipOd/2.0, 1.0)
	Ck(err)
	lip = sdf.Transform3D(lip, sdf.Translate3d(sdf.V3{0, 0, lipZ + wall/2.0}))

	res = sdf.Union3D(res, lip)

	pocket, err := sdf.Cylinder3D(body.Z, b.Id/2.0, 1.0)
	Ck(err)
	pocket = sdf.Transform3D(pocket, sdf.Translate3d(sdf.V3{0, 0, body.Z/2.0 + wall}))

	res = sdf.Difference3D(res, pocket)

	return
}

func main() {
	imuTop := wall + imu.Z
	imu.PostZ = imuTop - 5
	feather.PostZ = imuTop + feather.Z - 5
	feather.ShoulderZ = imuTop

	batteryBotZ := feather.PostZ
	battery.CornerZ = batteryBotZ + battery.Z

	coverZ := wall * 2
	body.Z = battery.CornerZ + coverZ/2.0
	res := body.shape()

	coverOd := hubOd
	coverId := body.Id

	bat := battery.shape()
	bat = sdf.Transform3D(bat, sdf.Translate3d(sdf.V3{0, 0, batteryBotZ + battery.Z/2.0}))

	corners := battery.corners()
	corners = sdf.Transform3D(corners, sdf.Translate3d(sdf.V3{0, 0, battery.CornerZ / 2.0}))
	corners = sdf.Difference3D(corners, bat)

	cover, err := sdf.Cylinder3D(coverZ/2.0, coverOd/2.0, 1.0)
	Ck(err)
	cover = sdf.Transform3D(cover, sdf.Translate3d(sdf.V3{0, 0, coverZ / 4.0}))
	covinner, err := sdf.Cylinder3D(coverZ/2.0, coverId/2.0, 0)
	Ck(err)
	covinner = sdf.Transform3D(covinner, sdf.Translate3d(sdf.V3{0, 0, -coverZ / 4.0}))
	cover = sdf.Union3D(cover, covinner)
	cover = sdf.Transform3D(cover, sdf.Translate3d(sdf.V3{0, 0, body.Z}))

	res = sdf.Union3D(res, feather.posts(), imu.posts(), corners)
	// res = sdf.Union3D(res, feather.posts(), imu.posts(), bat, corners)
	render.RenderSTL(res, 300, "body.stl")
	render.RenderSTL(cover, 300, "cover.stl")
}
