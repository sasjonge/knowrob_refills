/**  <module> refills

  Copyright (C) 2018 Daniel Beßler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

  @author Daniel Beßler
  @license BSD
*/

:- module(refills,
    [
      refills_init_test_shop/0,
      refills_init_iai_shop/0,
      refills_spawn_facings/0
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).

:- rdf_db:rdf_register_ns(dmshop, 'http://knowrob.org/kb/dm-market.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(shop, 'http://knowrob.org/kb/shop.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).

:- use_module(library('shop')).

refills_init_iai_shop :-
  owl_parser:owl_parse('package://knowrob_refills/owl/dm-market-iai.owl', belief_state).

refills_init_test_shop :-
  belief_parse('package://knowrob_refills/owl/shop-test.owl'),
  refills_spawn_facings.

refills_spawn_facings :-
  Frame1='http://knowrob.org/kb/shop-test.owl#DMShelfFrameFrontStore_5gKS',
  Frame2='http://knowrob.org/kb/shop-test.owl#DMShelfFrameFrontStore_Bnc8',
  belief_shelf_part_at(Frame1, dmshop:'DMShelfLayer4TilesFront', 0.9, _),
  belief_shelf_part_at(Frame1, dmshop:'DMShelfLayer4TilesFront', 0.7, _),
  belief_shelf_part_at(Frame1, dmshop:'DMShelfLayer4TilesFront', 0.5, _),
  belief_shelf_part_at(Frame1, dmshop:'DMShelfLayer4TilesFront', 0.2, _),
  belief_shelf_part_at(Frame2, dmshop:'DMShelfLayer4TilesFront', 0.1, _),
  belief_shelf_part_at(Frame2, dmshop:'DMShelfLayer4TilesFront', 0.3, _),
  belief_shelf_part_at(Frame2, dmshop:'DMShelfLayerMountingFront', 0.6, _),
  belief_shelf_part_at(Frame2, dmshop:'DMShelfLayerMountingFront', 1.0, _),
  forall( rdfs_individual_of(Layer, dmshop:'DMShelfLayerStanding'), (
    belief_shelf_part_at(Layer, dmshop:'DMShelfSeparator4Tiles', 0.0, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfSeparator4Tiles', 0.2, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfSeparator4Tiles', 0.35, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfSeparator4Tiles', 0.6, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfSeparator4Tiles', 0.85, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfSeparator4Tiles', 1.0, _),
    %belief_shelf_barcode_at(Layer, dmshop:'DMShelfLabel', dan('569070'), 0.1, _),
    %belief_shelf_barcode_at(Layer, dmshop:'DMShelfLabel', dan('538288'), 0.275, _),
    belief_shelf_barcode_at(Layer, dmshop:'DMShelfLabel', dan('438505'), 0.475, _),
    %belief_shelf_barcode_at(Layer, dmshop:'DMShelfLabel', dan('523915'), 0.725, _),
    belief_shelf_barcode_at(Layer, dmshop:'DMShelfLabel', dan('300941'), 0.925, _)
  )), 
  forall( rdfs_individual_of(Layer, dmshop:'DMShelfLayerMounting'), (
    belief_shelf_part_at(Layer, dmshop:'DMShelfMountingBar', 0.0, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfMountingBar', 0.1, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfMountingBar', 0.2, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfMountingBar', 0.3, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfMountingBar', 0.5, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfMountingBar', 0.7, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfMountingBar', 0.9, _),
    belief_shelf_part_at(Layer, dmshop:'DMShelfMountingBar', 1.0, _)
  )),
  forall(
    rdf_has(Facing, shop:labelOfFacing, _),(
    product_spawn_front_to_back(Facing,_),
    product_spawn_front_to_back(Facing,_))
  ),
  forall((
    rdf_has(Prev, shop:labelOfFacing, _),
    % TODO: this is odd, it should work the other way. Seems left/right mixed up?!?
    %rdf_has(Prev, shop:rightSeparator, X),
    %rdf_has(Facing, shop:leftSeparator, X)),
    rdf_has(Prev, shop:leftSeparator, X),
    rdf_has(Facing, shop:rightSeparator, X)),(
    product_spawn_front_to_back(Facing,_),
    product_spawn_front_to_back(Facing,_))
  ).
