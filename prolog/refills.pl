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
:- use_module(library('knowrob/temporal')).
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

refills_make_shelf(Frame, [(Pos,separators(Separators),labels(Labels))|Rest]) :-
  belief_shelf_part_at(Frame, dmshop:'DMShelfLayer4TilesFront', norm(Pos), Layer),
  forall(member(SepPos,Separators),
         belief_shelf_part_at(Layer,dmshop:'DMShelfSeparator4Tiles', norm(SepPos), _)),
  forall(member((LabelPos,AN),Labels),
         belief_shelf_barcode_at(Layer,dmshop:'DMShelfLabel',dan(AN),norm(LabelPos),_)),
  refills_make_shelf(Frame, Rest).

refills_make_shelf(Frame, [(Pos,bars(Bars),labels(Labels))|Rest]) :-
  belief_shelf_part_at(Frame, dmshop:'DMShelfLayerMountingFront', norm(Pos), Layer),
  forall(member(BarPos,Bars),
         belief_shelf_part_at(Layer,dmshop:'DMShelfMountingBar',norm(BarPos),_)),
  forall(member((LabelPos,AN),Labels),
         belief_shelf_barcode_at(Layer,dmshop:'DMShelfLabel',dan(AN),norm(LabelPos),_)),
  refills_make_shelf(Frame, Rest).

refills_make_shelf(_, []).

refills_random_facing(Facing) :-
  random(0,2,Full),
  random(0,5,Misplaced),
  
  shelf_facing_product_type(Facing, ProductType),
  once(( Misplaced>0 ; (
    rdfs_individual_of(OtherFacing, shop:'ProductFacing'),
    shelf_facing_product_type(OtherFacing, OtherProductType),
    OtherProductType \= ProductType,
    product_spawn_front_to_back(Facing,_,OtherProductType))
  )),
  
  ( Full>0 -> (
    product_spawn_front_to_back(Facing,_),
    standing_facing_full(Facing) ); (
    product_spawn_front_to_back(Facing,_),
    product_spawn_front_to_back(Facing,_),
    product_spawn_front_to_back(Facing,_),
    product_spawn_front_to_back(Facing,_)
  )).

standing_facing_full(Facing) :-
  product_spawn_front_to_back(Facing,_),
  ignore(standing_facing_full(Facing)).

refills_spawn_facings :-
  refills_make_shelf('http://knowrob.org/kb/shop-test.owl#DMShelfFrameFrontStore_5gKS', [
    (0.2, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'378940'),(0.925,'346864')])),
    (0.4, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'378981'),(0.925,'553736')])),
    (0.6, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'553735'),(0.925,'251188')])),
    (0.8, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'250899'),(0.925,'544382')]))
  ]),
  refills_make_shelf('http://knowrob.org/kb/shop-test.owl#DMShelfFrameFrontStore_Bnc8', [
    (0.1, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'553733'),(0.925,'404491')])),
    (0.3, separators([0.0,0.2,0.4,0.6,0.8,1.0]),  labels([(0.3,'522988'),(0.7,'402186'),(0.5,'520689'),(0.1,'476935'),(0.9,'004728')])),
    (0.8, bars([0.1,0.2,0.3,0.5,0.7,0.9]), labels([(0.2,'544382'),(0.9,'250899')])),
    (1.0, bars([0.1,0.2,0.3,0.5,0.7,0.9]), labels([(0.2,'250899'),(0.9,'544382')]))
  ]),
  forall(rdfs_individual_of(Facing, shop:'ProductFacing'),(
    random(0,6,HasProduct),
    (( HasProduct>0, rdf_has(Facing, shop:associatedLabelOfFacing, Label ) )-> (
       rdf_has(Label, shop:articleNumberOfLabel, ArticleNumber),
       ( refills_random_facing(Facing) -> true ; (
          write('[WARN] Failed to spawn products into '), owl_write_readable([Facing,ArticleNumber]), nl ))
    ) ; true)
  )).
