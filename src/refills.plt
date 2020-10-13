:- begin_tests(refills).

:- register_ros_package(knowrob_refills).

:- beliefstate:belief_parse('package://knowrob_refills/owl/shop-test.owl').
:- owl_parser:owl_parse('package://knowrob_refills/owl/product-catalog.owl').

:- use_module(library('shop')).
:- use_module(library('refills')).
:- use_module(library(statistics)).

frame1('http://knowrob.org/kb/shop-test.owl#DMShelfFrameFrontStore_5gKS').
frame2('http://knowrob.org/kb/shop-test.owl#DMShelfFrameFrontStore_Bnc8').

test(shelf_spawn_layer) :-
  frame1(Frame1),
  frame2(Frame2),
  shelf_frame_spawn_layer(Frame1, dmshop:'DMShelfLayer4TilesFront', 0.9, _),
  shelf_frame_spawn_layer(Frame1, dmshop:'DMShelfLayer4TilesFront', 0.7, _),
  shelf_frame_spawn_layer(Frame1, dmshop:'DMShelfLayer4TilesFront', 0.5, _),
  shelf_frame_spawn_layer(Frame1, dmshop:'DMShelfLayer4TilesFront', 0.2, _),
  shelf_frame_spawn_layer(Frame2, dmshop:'DMShelfLayer4TilesFront', 0.1, _),
  shelf_frame_spawn_layer(Frame2, dmshop:'DMShelfLayer4TilesFront', 0.3, _),
  shelf_frame_spawn_layer(Frame2, dmshop:'DMShelfLayerMountingFront', 0.6, _),
  shelf_frame_spawn_layer(Frame2, dmshop:'DMShelfLayerMountingFront', 1.0, _).

test(shelf_spawn_separator) :-
  forall( rdfs_individual_of(Layer, dmshop:'DMShelfLayerStanding'), (
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.0, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.2, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.35, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.6, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 0.85, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfSeparator', 1.0, _)
  )).

test(shelf_spawn_label) :-
  forall( rdfs_individual_of(Layer, dmshop:'DMShelfLayerStanding'), (
    shelf_layer_spawn_label(Layer, dmshop:'DMShelfLabel', dan('438505'), 0.475, _),
    shelf_layer_spawn_label(Layer, dmshop:'DMShelfLabel', dan('300941'), 0.925, _)
  )).

test(shelf_spawn_mounting_bars) :-
  forall( rdfs_individual_of(Layer, dmshop:'DMShelfLayerMounting'), (
    shelf_layer_spawn(Layer, dmshop:'DMShelfMountingBar', 0.0, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfMountingBar', 0.1, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfMountingBar', 0.2, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfMountingBar', 0.3, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfMountingBar', 0.5, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfMountingBar', 0.7, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfMountingBar', 0.9, _),
    shelf_layer_spawn(Layer, dmshop:'DMShelfMountingBar', 1.0, _)
  )).

:- end_tests(refills).
