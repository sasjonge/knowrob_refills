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
      %refills_init_iai_shop/0,
      refills_spawn_facings/0,
      refills_spawn_products/0,
      refills_test_spawning60(r),
      refills_spawn_shelf1(r),
      refills_spawn_shelf2(r),
      refills_make_shelf(r,t),
      shelf_floor_type(r,r),
      shelf_bottom_floor_type(r,r),
      refills_test_spawning(r),
      refills_test_spawning(r,+),
      bulk_insert_floor(r,+,+)
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('model/SOMA/OBJ')).
:- use_module(library('lang/computable')).
:- use_module(library('lang/terms/is_a')).
:- use_module(library('lang/query')).
:- use_module(library('lang/terms/temporal')).
:- use_module(library('db/tripledb')).
:- use_module(library('ros/tf/tf_plugin')).


:- rdf_db:rdf_register_ns(dmshop, 'http://knowrob.org/kb/dm-market.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(shop, 'http://knowrob.org/kb/shop.owl#', [keep(true)]).
%% :- tripledb_load('package://knowrob_refills/owl/shop-test.owl', 
%%   [namespace(shoptest,'http://knowrob.org/kb/shop-test.owl#')] ).
:- tripledb_load('package://knowrob_refills/owl/product-catalog.owl').

:- use_module(library('shop')).

shelf_bottom_floor_type(Frame,Type) :-
  instance_of(Frame,FrameType),
  shelf_bottom_floor_type_(FrameType,Type), !.
shelf_bottom_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT5')),
  subclass_of(FrameType,dmshop:'DMShelfW100'),
  rdf_equal(Type,dmshop:'DMBFloorT5W100'),!.
shelf_bottom_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT5')),
  subclass_of(FrameType,dmshop:'DMShelfW60'),
  rdf_equal(Type,dmshop:'DMBFloorT5W60'),!.
shelf_bottom_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT6')),
  subclass_of(FrameType,dmshop:'DMShelfW60'),
  rdf_equal(Type,dmshop:'DMBFloorT6W60'),!.
shelf_bottom_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT6')),
  subclass_of(FrameType,dmshop:'DMShelfW70'),
  rdf_equal(Type,dmshop:'DMBFloorT6W75'),!.
shelf_bottom_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT6')),
  subclass_of(FrameType,dmshop:'DMShelfW100'),
  rdf_equal(Type,dmshop:'DMBFloorT6W100'),!.
shelf_bottom_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT6')),
  subclass_of(Frame,dmshop:'DMShelfW120'),
  rdf_equal(Type,dmshop:'DMBFloorT6W120'),!.
shelf_bottom_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT7')),
  subclass_of(FrameType,dmshop:'DMShelfW100'),
  rdf_equal(Type,dmshop:'DMBFloorT7W100'),!.
shelf_bottom_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT7')),
  subclass_of(FrameType,dmshop:'DMShelfW120'),
  rdf_equal(Type,dmshop:'DMBFloorT7W120'),!.

shelf_floor_type(Frame,Type) :-
  instance_of(Frame,FrameType),
  shelf_floor_type_(FrameType,Type), !.
shelf_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT5')),
  subclass_of(FrameType,dmshop:'DMShelfW100'),
  rdf_equal(Type,dmshop:'DMFloorT4W100'),!.
shelf_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT5')),
  subclass_of(FrameType,dmshop:'DMShelfW60'),
  rdf_equal(Type,dmshop:'DMFloorT5W60'),!.
shelf_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT6')),
  subclass_of(FrameType,dmshop:'DMShelfW60'),
  rdf_equal(Type,dmshop:'DMFloorT5W60'),!.
shelf_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT6')),
  subclass_of(FrameType,dmshop:'DMShelfW70'),
  rdf_equal(Type,dmshop:'DMFloorT5W75'),!.
shelf_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT6')),
  subclass_of(FrameType,dmshop:'DMShelfW100'),
  rdf_equal(Type,dmshop:'DMFloorT5W100'),!.
shelf_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT6')),
  subclass_of(FrameType,dmshop:'DMShelfW120'),
  rdf_equal(Type,dmshop:'DMFloorT5W120'),!.
shelf_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT7')),
  subclass_of(FrameType,dmshop:'DMShelfW100'),
  rdf_equal(Type,dmshop:'DMFloorT6W100'),!.
shelf_floor_type_(FrameType,Type) :-
  transitive(subclass_of(FrameType,dmshop:'DMShelfT7')),
  subclass_of(FrameType,dmshop:'DMShelfW120'),
  rdf_equal(Type,dmshop:'DMFloorT6W120'),!.

%refills_init_iai_shop :-
  %tripledb_load('package://knowrob_refills/owl/dm-market-iai.owl').

refills_init_test_shop :-
  refills_spawn_facings.

bulk_insert_floor(Floor, separators(Separators), labels(Labels)) :-
  forall(
    member(SepPos,Separators),
    belief_shelf_part_at(Floor,'http://knowrob.org/kb/dm-market.owl#DMShelfSeparator4Tiles', norm(SepPos), _, [insert])),
  forall(
    member((LabelPos,AN),Labels),
    belief_shelf_barcode_at(Floor,'http://knowrob.org/kb/dm-market.owl#DMShelfLabel',dan(AN),norm(LabelPos), _, [insert])),
  % update facings 
  shelf_facings_mark_dirty(Floor).

% @deprecated
refills_make_shelf_old(Frame, [(Pos,separators(Separators),labels(Labels))|Rest]) :-
  shelf_floor_type(Frame,FloorType),
  belief_shelf_part_at(Frame, FloorType, norm(Pos), Layer),
  % assert to belief state
  forall(member(SepPos,Separators),
         belief_shelf_part_at(Layer,'http://knowrob.org/kb/dm-market.owl#DMShelfSeparator4Tiles', norm(SepPos), _)),
  forall(member((LabelPos,AN),Labels),
         belief_shelf_barcode_at(Layer,'http://knowrob.org/kb/dm-market.owl#DMShelfLabel',dan(AN),norm(LabelPos),_)),
  refills_make_shelf(Frame, Rest).

% @deprecated
refills_make_shelf_old(Frame, [(separators(Separators),labels(Labels))|Rest]) :-
  Pos is 0.075,
  shelf_bottom_floor_type(Frame,BottomFloorType),
  belief_shelf_part_at(Frame, BottomFloorType, norm(Pos), Layer),
  forall(member(SepPos,Separators),
         belief_shelf_part_at(Layer,'http://knowrob.org/kb/dm-market.owl#DMShelfSeparator4Tiles', norm(SepPos), _)),
  forall(member((LabelPos,AN),Labels),
         belief_shelf_barcode_at(Layer,'http://knowrob.org/kb/dm-market.owl#DMShelfLabel',dan(AN),norm(LabelPos),_)),
  refills_make_shelf(Frame, Rest).

refills_make_shelf(Frame, [(Pos,bars(Bars),labels(Labels))|Rest]) :-
  !,
  belief_shelf_part_at(Frame, 'http://knowrob.org/kb/dm-market.owl#DMShelfLayerMountingFront', norm(Pos), Layer),
  forall(member(BarPos,Bars),
         belief_shelf_part_at(Layer,'http://knowrob.org/kb/dm-market.owl#DMShelfMountingBar',norm(BarPos),_, [insert])),
  forall(member((LabelPos,AN),Labels),
         belief_shelf_barcode_at(Layer,'http://knowrob.org/kb/dm-market.owl#DMShelfLabel', dan(DAN),norm(LabelPos),_, [insert])),
  refills_make_shelf(Frame, Rest).

refills_make_shelf(Frame, [(Pos,Separators,Labels)|Rest]) :-
  !,
  shelf_floor_type(Frame,FloorType),
  belief_shelf_part_at(Frame, FloorType, norm(Pos), Layer),
  bulk_insert_floor(Layer,Separators,Labels),
  refills_make_shelf(Frame, Rest).

refills_make_shelf(Frame, [(Separators,Labels)|Rest]) :-
  Pos is 0.075,
  shelf_bottom_floor_type(Frame,BottomFloorType),
  belief_shelf_part_at(Frame, BottomFloorType, norm(Pos), Layer),
  bulk_insert_floor(Layer,Separators,Labels),
  %bulk_insert_floor('http://knowrob.org/kb/shop-test.owl#shelfLayer_test',Separators,Labels),
  refills_make_shelf(Frame, Rest).

refills_make_shelf(_, []).

%refills_random_facing(Facing) :-
  %comp_isSpaceRemainingInFacing(Facing, literal(type(_,false))), !.

refills_random_facing(Facing) :-
  tripledb_forget(Facing, shop:'productInFacing', _),
  random(0,6,Full),
  % random(0,5,Misplaced),
  Misplaced is 4,
  once(shelf_facing_product_type(Facing, ProductType)),
  once(( Misplaced>0 ; (
    instance_of(OtherFacing, shop:'ProductFacing'),
    shelf_facing_product_type(OtherFacing, OtherProductType),
    OtherProductType \= ProductType,
    product_spawn_front_to_back(Facing,_,OtherProductType))
  )),
  ( Full>0 ->
    standing_facing_full(Facing); 
    ignore(product_spawn_front_to_back(Facing,_))
  ).

standing_facing_full(Facing) :-
  comp_isSpaceRemainingInFacing(Facing, literal(type(_,false))), !.

standing_facing_full(Facing) :-
  product_spawn_front_to_back(Facing,_),
  standing_facing_full(Facing).

refills_spawn_products :-
  forall(instance_of(Facing, shop:'ProductFacing'),(
    %random(0,6,HasProduct),
    HasProduct is 1,
    (( HasProduct>0, comp_preferredLabelOfFacing(Facing,Label) )-> (
       holds(Label, shop:articleNumberOfLabel, AN),
       ( refills_random_facing(Facing) -> true ; (
          write('[WARN] Failed to spawn products into '), write([Facing,AN]), nl ))
    ) ; true)
  )).

refills_spawn_shelf1(Shelf) :-
  refills_make_shelf(Shelf, [
    (separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.21,'378940'),(0.5,'402186'),(0.925,'346864')])),
    (0.4, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'378981'),(0.925,'553736')])),
    (0.6, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'553735'),(0.925,'251188')])),
    (0.8, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'250899'),(0.925,'544382')]))
  ]).
  
refills_spawn_shelf2(Shelf) :-
  refills_make_shelf(Shelf, [
    (0.1, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.4,'553733'),(0.925,'404491')])),
    (0.3, separators([0.0,0.2,0.4,0.6,0.8,1.0]),  labels([(0.3,'522988'),(0.7,'402186'),(0.5,'520689'),(0.1,'476935'),(0.9,'004728')])),
    (0.8, bars([0.1,0.2,0.3,0.5,0.7,0.9]), labels([(0.2,'544382'),(0.9,'250899')])),
    (1.0, bars([0.1,0.2,0.3,0.5,0.7,0.9]), labels([(0.2,'250899'),(0.9,'544382')]))
  ]).

refills_spawn_facings :-
  tell(is_at(dmshop:'Pose_T5_T4_L', ['map',[0.0,0.01,-0.095],[0.0,0.0,0.0,1.0]])),
  tell(is_at(shoptest:'DMShelfFrameFrontStore_5gKS', ['map', [0.0569,2.0474,0.8532], [0.0,0.0,0.7071,0.7071]])),
  tell(is_at(shoptest:'DMShelfFrameFrontStore_Bnc8', ['map', [0.0603,1.0045,0.8559], [0.0,0.0,0.7071,0.7071]])),
  tell(holds(shoptest:'DMShelfFrameFrontStore_Bnc8', knowrob:'frameName', 'DMShelfFrameFrontStore_Bnc8')),
  tell(holds(shoptest:'DMShelfFrameFrontStore_5gKS', knowrob:'frameName', 'DMShelfFrameFrontStore_5gKS')),
  refills_spawn_shelf1('http://knowrob.org/kb/shop-test.owl#DMShelfFrameFrontStore_5gKS'),
  refills_spawn_shelf2('http://knowrob.org/kb/shop-test.owl#DMShelfFrameFrontStore_Bnc8'),
  refills_spawn_products.

%%%%%%%%

shelf_type(Type) :-
  rdfs_subclass_of(Type,shop:'ShelfFrame'),
  once( owl_class_properties(Type, knowrob:pathToCadModel, _) ).

shelf_fill(Shelf,Data) :-
  current_time(T0),
  refills_make_shelf(Shelf,Data),
  current_time(T1),
  Diff is T1 - T0,
  write('    spawning floors took: '), write(Diff), writeln(' s').

refills_test_spawning60(Shelf) :-
  S15=[0.06,0.12,0.18,0.24,0.3,0.36,0.42,0.48,0.54,0.6,0.66,0.72,0.78,0.84,1.0],
  L5=[(0.12,'378940'),(0.36,'402186'),(0.6,'346864'),(0.72,'553736'),(0.84,'250899')],
  refills_test_spawning(Shelf, [
    (     separators(S15), labels(L5)),
    (0.4, separators(S15), labels(L5)),
    (0.6, separators(S15), labels(L5)),
    (0.8, separators(S15), labels(L5))
  ]).

refills_test_spawning(Shelf) :-
  refills_test_spawning(Shelf, [
    (separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.21,'378940'),(0.5,'402186'),(0.925,'346864')])),
    (0.4, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'378981'),(0.925,'553736')])),
    (0.6, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'553735'),(0.925,'251188')])),
    (0.8, separators([0.0,0.2,0.4,0.6,0.85,1.0]), labels([(0.475,'250899'),(0.925,'544382')]))
  ]).

refills_test_spawning(Shelf,Data) :-
  shelf_type(ShelfType),
  % remove old marker
  write('cleaning up...'),nl,
  belief_forget,
  %reload_objects,
  sleep(2.0),
  write('shelf type: '), owl_write_readable(ShelfType), nl,
  belief_new_object(ShelfType,Shelf),
  object_dimensions(Shelf,_,_,ShelfH),
  ShelfH2 is 0.5*ShelfH,
  belief_at_update(Shelf,[map,_,[0.0,0.0,ShelfH2],[0,0,0,1]]),
  writeln('  spawning floors...'),
  ( shelf_fill(Shelf,Data) ->
    writeln('    done.') ;
    writeln('    ERROR something went wrong!') ),
  writeln('  spawning objects...'),
  refills_spawn_products,
  writeln('done.').
  

%%%%%%%%
%%
%%  Unit Tests
%%  
%%%%%%%%
  
:- begin_tripledb_tests(
    'refills',
    'package://knowrob_refills/owl/shop.owl',
    [ namespace('http://knowrob.org/kb/shop.owl#')
    ]).

test('init'):-
  %refills_init_iai_shop,
  refills_init_test_shop.

:- end_tripledb_tests('refills').
