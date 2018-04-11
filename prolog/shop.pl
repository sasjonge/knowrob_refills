/**  <module> shop

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

% TODO:
%    - add conversation hook for ean/dan?
%    - use qudt and ensure unit is meters here.
%    - include information about how much space can be taken by objects in layers

:- module(shop,
    [
      shelf_layer_frame/2,
      shelf_layer_mounting/1,
      shelf_layer_standing/1,
      shelf_layer_above/2,
      shelf_layer_below/2,
      shelf_layer_position/3,
      shelf_layer_separator/2,
      shelf_layer_mounting_bar/2,
      shelf_layer_label/2,
      shelf_facing/2,
      shelf_facing_product_type/2,
      % computable properties
      comp_isSpaceRemainingInFacing/2,
      comp_facingPose/2,
      comp_facingWidth/2,
      comp_facingHeight/2,
      comp_facingDepth/2,
      comp_preferredLabelOfFacing/2,
      %%%%%
      belief_shelf_part_at/4,
      belief_shelf_barcode_at/5,
      product_spawn_front_to_back/2,
      product_spawn_front_to_back/3
    ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/owl')).

:-  rdf_meta
    shelf_layer_frame(r,r),
    shelf_layer_above(r,r),
    shelf_layer_below(r,r),
    shelf_layer_mounting(r),
    shelf_layer_standing(r),
    shelf_layer_position(r,r,-),
    shelf_layer_mounting_bar(r,r),
    shelf_layer_label(r,r),
    shelf_layer_separator(r,r),
    shelf_facing_product_type(r,r),
    shelf_layer_part(r,r,r),
    belief_shelf_part_at(r,r,+,-),
    belief_shelf_barcode_at(r,r,+,+,-),
    product_type_dimension_assert(r,r,+).

:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).
:- rdf_db:rdf_register_ns(shop, 'http://knowrob.org/kb/shop.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob_assembly, 'http://knowrob.org/kb/knowrob_assembly.owl#', [keep(true)]).

% TODO: should be somewhere else
% TODO: must work in both directions
xsd_float(Value, literal(
    type('http://www.w3.org/2001/XMLSchema#float', Atom))) :-
  atom(Value) -> Atom=Value ; atom_number(Atom,Value).
xsd_boolean(Atom, literal(
    type('http://www.w3.org/2001/XMLSchema#boolean', Atom))) :-
  atom(Atom).

prolog:message(shop(Entities,Msg)) -->
  { owl_readable(Entities,Entities_readable) },
  ['[shop.pl] ~w ~w'-[Msg,Entities_readable]].
  
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ArticleNumber'

create_article_number(ean(AN), ArticleNumber) :-
  create_article_number_(literal(type('http://knowrob.org/kb/shop.owl#ean',AN)), ArticleNumber).
create_article_number(dan(AN), ArticleNumber) :-
  create_article_number_(literal(type('http://knowrob.org/kb/shop.owl#dan',AN)), ArticleNumber).
create_article_number_(AN_XSD,ArticleNumber) :-
  owl_has(ArticleNumber, shop:articleNumberString, AN_XSD), !.
create_article_number_(AN_XSD, ArticleNumber) :-
  strip_literal_type(AN_XSD, AN_atom),
  atomic_list_concat([
    'http://knowrob.org/kb/shop.owl#ArticleNumber_',
    AN_atom], ArticleNumber),
  rdf_assert(ArticleNumber, rdf:type, shop:'ArticleNumber'),
  rdf_assert(ArticleNumber, rdf:type, owl:'NamedIndividual'),
  rdf_assert(ArticleNumber, shop:articleNumberString, AN_XSD),
  create_product_type(ArticleNumber, ProductType),
  print_message(warning, shop([ProductType], 'Missing product type. Incomplete data?')).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'Product'

create_product_type(ArticleNumber,ProductType) :-
  rdf_has(ArticleNumber, shop:articleNumberString, literal(type(_,AN_atom))),
  atomic_list_concat([
    'http://knowrob.org/kb/shop.owl#ProductWithAN',
    AN_atom], ProductType),
  rdf_assert(ProductType, rdfs:subClassOf, shop:'Product'),
  rdf_assert(ProductType, rdf:type, owl:'Class'),
  owl_restriction_assert(restriction(
    shop:articleNumberOfProduct,
    has_value(ArticleNumber)), AN_R),
  rdf_assert(ProductType, rdfs:subClassOf, AN_R).

product_type_dimensions(Type, [D,W,H]) :-
  owl_property_range_on_class(Type, shop:depthOfProduct,  literal(type(_,D_atom))),
  owl_property_range_on_class(Type, shop:widthOfProduct,  literal(type(_,W_atom))),
  owl_property_range_on_class(Type, shop:heightOfProduct, literal(type(_,H_atom))),
  atom_number(D_atom, D),
  atom_number(W_atom, W),
  atom_number(H_atom, H), !.
product_type_dimensions(Type, [0.04,0.04,0.04]) :-
  print_message(warning, shop(Type,'No bounding box is defined')),
  product_type_dimension_assert(Type, shop:depthOfProduct, 0.04),
  product_type_dimension_assert(Type, shop:widthOfProduct, 0.04),
  product_type_dimension_assert(Type, shop:heightOfProduct, 0.04), !.

product_type_dimension_assert(Type, P, Val) :-
  xsd_float(Val,Val_XSD),
  owl_restriction_assert(restriction(P, has_value(Val_XSD)), R),
  rdf_assert(Type, rdfs:subClassOf, R),
  owl:owl_property_range_clear_cache(Type,P).


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLayer'

%% 
shelf_layer_mounting(ShelfLayer) :- rdfs_individual_of(ShelfLayer, shop:'ShelfLayerMounting').
%% 
shelf_layer_standing(ShelfLayer) :- rdfs_individual_of(ShelfLayer, shop:'ShelfLayerStanding').

%% 
shelf_layer_frame(Layer, Frame) :-
  owl_has(Frame, knowrob:properPhysicalParts, Layer),
  rdfs_individual_of(Frame, shop:'ShelfFrame'), !.
%%
shelf_facing(ShelfLayer, Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer).

%%
shelf_layer_separator(Layer,Part)    :- shelf_layer_part(Layer,shop:'ShelfSeparator',Part).
%%
shelf_layer_mounting_bar(Layer,Part) :- shelf_layer_part(Layer,shop:'ShelfMountingBar',Part).
%%
shelf_layer_label(Layer,Part)        :- shelf_layer_part(Layer,shop:'ShelfLabel',Part).
%%
shelf_layer_part(Layer, Type, Part) :-
  rdfs_individual_of(Layer, shop:'ShelfLayer'),
  owl_has(Layer, knowrob:properPhysicalParts, Part),
  rdfs_individual_of(Part, Type).

%% shelf_layer_position
%
% The position of some object on a shelf layer.
% Position is simply the x-value of the object's pose in the shelf layer's frame.
%
shelf_layer_position(Layer, Object, Position) :-
  belief_at_relative_to(Object, Layer, [_,_,[Position,_,_],_]).

%%
shelf_facing_position(Facing,Pos) :-
  rdf_has(Facing, shop:leftSeparator, Left), !,
  rdf_has(Facing, shop:rightSeparator, Right),
  rdf_has(Facing, shop:layerOfFacing, Layer),
  shelf_layer_position(Layer, Left, Pos_Left),
  shelf_layer_position(Layer, Right, Pos_Right),
  Pos is 0.5*(Pos_Left+Pos_Right).
shelf_facing_position(Facing, Pos) :-
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar), !,
  rdf_has(Facing, shop:layerOfFacing, Layer),
  shelf_layer_position(Layer, MountingBar, Pos).

%% 
shelf_layer_above(ShelfLayer, AboveLayer) :-
  shelf_layer_sibling(ShelfLayer, min_positive_element, AboveLayer).
%% 
shelf_layer_below(ShelfLayer, BelowLayer) :-
  shelf_layer_sibling(ShelfLayer, max_negative_element, BelowLayer).

shelf_layer_sibling(ShelfLayer, Selector, SiblingLayer) :-
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  belief_at_relative_to(ShelfLayer, ShelfFrame, [_,_,[_,_,Pos],_]),
  findall((X,Diff), (
    rdf_has(ShelfFrame, knowrob:properPhysicalParts, X),
    X \= ShelfLayer,
    belief_at_relative_to(X, ShelfFrame, [_,_,[_,_,X_Pos],_]),
    Diff is X_Pos-Pos), Xs),
  call(Selector, Xs, (SiblingLayer,_)).

shelf_layer_neighbours(ShelfLayer, Needle, Selector, Positions) :-
  shelf_layer_position(ShelfLayer, Needle, NeedlePos),
  findall((X,D), (
    call(Selector, ShelfLayer, X),
    X \= Needle,
    shelf_layer_position(ShelfLayer, X, Pos_X),
    D is Pos_X - NeedlePos
  ), Positions).

shelf_facing_labels_update(ShelfLayer,Facing) :-
  forall((
    owl_has(Facing,shop:adjacentLabelOfFacing,Label);
    rdf_has(Facing,shop:labelOfFacing,Label)),(
    shelf_label_insert(ShelfLayer,Label)
  )).

shelf_facings_mark_dirty(ShelfLayer) :-
  findall(X, shelf_facing(ShelfLayer,X), AllFacings),
  belief_republish_objects(AllFacings).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfSeparator'

%%
shelf_separator_insert(ShelfLayer,Separator) :-
  shelf_layer_standing(ShelfLayer),
  % [X.pos - Separator.pos]
  shelf_layer_neighbours(ShelfLayer, Separator, shelf_layer_separator, Xs),
  ignore(max_negative_element(Xs, (LeftOf,_))),
  ignore(min_positive_element(Xs, (RightOf,_))),
  %
  (( rdf_has(_,shop:rightSeparator,Separator) ;
     rdf_has(_,shop:leftSeparator,Separator)) ->
     shelf_separator_update(ShelfLayer, Separator, [LeftOf,RightOf]);
     shelf_separator_add(   ShelfLayer, Separator, [LeftOf,RightOf]) ),
  % update labelOfFacing relation
  ((rdf_has(Y,shop:leftSeparator,Separator);
    rdf_has(Y,shop:rightSeparator,Separator)) ->
    shelf_facing_labels_update(ShelfLayer,Y) ; true ),
  shelf_facings_mark_dirty(ShelfLayer).
   
shelf_separator_update(_, X, [LeftOf,RightOf]) :-
  % LeftOf,RightOf unchanged if ...
  % FIXME: could be that productInFacing changes, but highly unlikely.
  %        this is at the moment only updated when a facing is retracted.
  ( rdf_has(Facing1,shop:rightSeparator,X) ->
  ( ground(LeftOf), rdf_has(Facing1,shop:leftSeparator,LeftOf));    \+ ground(LeftOf)),
  ( rdf_has(Facing2,shop:leftSeparator,X) ->
  ( ground(RightOf), rdf_has(Facing2,shop:rightSeparator,RightOf)); \+ ground(RightOf)), !.
shelf_separator_update(Layer, X, [LeftOf,RightOf]) :-
  % LeftOf,RightOf changed otherwise
  shelf_separator_remove(X),
  shelf_separator_add(Layer, X, [LeftOf,RightOf]).

shelf_separator_add(Layer, X, [LeftOf,RightOf]) :-
  (ground(RightOf) -> shelf_facing_assert(Layer,[X,RightOf],_) ; true),
  (ground(LeftOf)  -> shelf_facing_assert(Layer,[LeftOf,X],_) ; true),
  ((ground([RightOf,LeftOf]),
    rdf_has(Facing, shop:rightSeparator,RightOf),
    rdf_has(Facing, shop:leftSeparator,LeftOf)) ->
    shelf_facing_retract(Facing) ; true ).

shelf_separator_remove(X) :-
  ( rdf_has(LeftFacing,shop:rightSeparator,X) ->
    shelf_separator_remove_rightSeparator(LeftFacing,X); true ),
  ( rdf_has(RightFacing,shop:leftSeparator,Y) ->
    shelf_separator_remove_leftSeparator(RightFacing,Y); true ).
shelf_separator_remove_rightSeparator(Facing,X) :-
  rdf_has(Facing,shop:leftSeparator,Left),
  rdf_has(NextFacing,shop:leftSeparator,X),
  rdf_retractall(NextFacing,shop:leftSeparator,X),
  rdf_assert(NextFacing,shop:leftSeparator,Left),
  shelf_facing_retract(Facing).
shelf_separator_remove_leftSeparator(Facing,X) :-
  rdf_has(Facing,shop:rightSeparator,Right),
  rdf_has(NextFacing,shop:rightSeparator,X),
  rdf_retractall(NextFacing,shop:rightSeparator,X),
  rdf_assert(NextFacing,shop:rightSeparator,Right),
  shelf_facing_retract(Facing).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfMountingBar'

shelf_mounting_bar_insert(ShelfLayer,MountingBar) :-
  (( rdf_has(Facing,shop:mountingBarOfFacing,MountingBar),
     shelf_mounting_bar_remove(MountingBar) );
     shelf_facing_assert(ShelfLayer,MountingBar,Facing)), !,
  % [X.pos - MountingBar.pos]
  shelf_layer_neighbours(ShelfLayer, MountingBar, shelf_layer_mounting_bar, Xs),
  ( min_positive_element(Xs, (X,_)) -> (
    % positive means that X is right of Separator
    rdf_assert(Facing, shop:rightMountingBar, X, belief_state),
    rdf_has(RightFacing, shop:mountingBarOfFacing, X),
    rdf_retractall(RightFacing, shop:leftMountingBar, _),
    rdf_assert(RightFacing, shop:leftMountingBar, MountingBar, belief_state)) ;
    true ),
  ( max_negative_element(Xs, (Y,_)) -> (
    % negative means that Y is left of Separator
    rdf_assert(Facing, shop:leftMountingBar, Y, belief_state),
    rdf_has(LeftFacing, shop:mountingBarOfFacing, Y),
    rdf_retractall(LeftFacing, shop:rightMountingBar, _),
    rdf_assert(LeftFacing, shop:rightMountingBar, MountingBar, belief_state)) ;
    true ),
  % update labelOfFacing and productInFacing relations
  shelf_facing_labels_update(ShelfLayer,Facing),
  shelf_facings_mark_dirty(ShelfLayer).

shelf_mounting_bar_remove(MountingBar) :-
  rdf_has(Facing,shop:mountingBarOfFacing,MountingBar),
  (( rdf_has(Facing, shop:rightMountingBar, Right),
     rdf_has(Facing, shop:leftMountingBar, Left) ) -> (
     rdf_has(RightFacing, shop:mountingBarOfFacing, Right),
     rdf_has(LeftFacing, shop:mountingBarOfFacing, Left),
     rdf_assert(RightFacing,shop:leftMountingBar,Left),
     rdf_assert(LeftFacing,shop:rightMountingBar,Right)
  );true),
  rdf_retractall(_,shop:leftMountingBar,MountingBar),
  rdf_retractall(_,shop:rightMountingBar,MountingBar).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLabel'

shelf_label_insert(ShelfLayer,Label) :-
  rdf_retractall(_,shop:labelOfFacing,Label),
  % find the position of Label on the shelf
  shelf_layer_position(ShelfLayer, Label, LabelPos),
  owl_has_prolog(Label, knowrob:widthOfObject, LabelWidth),
  LabelPosLeft  is LabelPos - 0.5*LabelWidth,
  LabelPosRight is LabelPos + 0.5*LabelWidth,
  % first find the facing under which the label was perceived, 
  % then assert labelOfFacing relation
  ( shelf_layer_find_facing_at(ShelfLayer,LabelPosLeft,LabeledFacingLeft) ->
    rdf_assert(LabeledFacingLeft,shop:labelOfFacing,Label,belief_state) ;
    true ),
  ((shelf_layer_find_facing_at(ShelfLayer,LabelPosRight,LabeledFacingRight),
    LabeledFacingRight \= LabeledFacingLeft ) ->
    rdf_assert(LabeledFacingRight,shop:labelOfFacing,Label,belief_state) ;
    true ),
  shelf_facings_mark_dirty(ShelfLayer).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ProductFacing'

shelf_facing_assert(ShelfLayer,[Left,Right],Facing) :-
  shelf_layer_standing(ShelfLayer), !,
  rdfs_individual_of(Left, shop:'ShelfSeparator'),
  rdfs_individual_of(Right, shop:'ShelfSeparator'),
  rdf_instance_from_class(shop:'ProductFacingStanding', belief_state, Facing),
  rdf_assert(Facing, shop:leftSeparator, Left, belief_state),
  rdf_assert(Facing, shop:rightSeparator, Right, belief_state),
  rdf_assert(Facing, shop:layerOfFacing, ShelfLayer, belief_state).

shelf_facing_assert(ShelfLayer,MountingBar,Facing) :-
  shelf_layer_mounting(ShelfLayer), !,
  rdfs_individual_of(MountingBar, shop:'ShelfMountingBar'),
  rdf_instance_from_class(shop:'ProductFacingMounting', belief_state, Facing),
  rdf_assert(Facing, shop:mountingBarOfFacing, MountingBar, belief_state),
  rdf_assert(Facing, shop:layerOfFacing, ShelfLayer, belief_state).

shelf_facing_retract(Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  rdf_retractall(Facing, shop:layerOfFacing, _),
  % remove products from facing
  forall(rdf_has(Product,shop:productInFacing,Facing),
         shelf_layer_insert_product(ShelfLayer,Product)),
  % TODO: don't forget about this facing, also marker remve msg should be generated
  rdf_retractall(Facing, _, _).

shelf_layer_insert_product(ShelfLayer,Product) :-
  shelf_layer_position(ShelfLayer,Product,Pos_Product),
  ( shelf_layer_find_facing_at(ShelfLayer,Pos_Product,Facing) ->
    rdf_assert(Facing, shop:productInFacing, Product, belief_state) ;
    true ).

shelf_layer_find_facing_at(ShelfLayer,Pos,Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_facing_position(Facing,FacingPos),
  shelf_facing_width(Facing, FacingWidth),
  Width is FacingWidth+0.02,
  FacingPos-0.5*Width =< Pos, Pos =< FacingPos+0.5*Width, !.

facing_space_remaining_in_front(Facing,Obj) :-
  belief_at_id(Obj, [_,_,[_,Obj_pos,_],_]),
  product_dimensions(Obj,Obj_depth,_,_),
  object_dimensions(Facing,Facing_depth,_,_),
  Obj_pos > Obj_depth + 0.06 - 0.5*Facing_depth.

facing_space_remaining_behind(Facing,Obj) :-
  belief_at_id(Obj, [_,_,[_,Obj_pos,_],_]),
  product_dimensions(Obj,Obj_depth,_,_),
  object_dimensions(Facing,Facing_depth,_,_),
  Obj_pos < Facing_depth*0.5 - Obj_depth - 0.06.

%% shelf_facing_product_type
%
shelf_facing_product_type(Facing, ProductType) :-
  holds(Facing, shop:articleNumberOfFacing, ArticleNumber),
  rdf_has(R, owl:hasValue, ArticleNumber),
  rdf_has(R, owl:onProperty, shop:articleNumberOfProduct),
  rdf_has(ProductType, rdfs:subClassOf, R),
  rdf_has(ProductType, rdf:type, owl:'Class'), !.
shelf_facing_product_type(Facing, _) :-
  print_message(warning, shop([Facing], 'Facing has no associated product type.')),
  fail.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% COMPUTABLES

%% comp_preferredLabelOfFacing
%
comp_preferredLabelOfFacing(Facing,Label) :-
  ground(Facing),
  % preferred if label is one of the labelOfFacing
  findall(L,rdf_has(Facing,shop:labelOfFacing,L),[X|Xs]), !,
  member(Label,[X|Xs]).
comp_preferredLabelOfFacing(Facing,Label) :-
  ground(Facing), !,
  owl_has(Facing,shop:adjacentLabelOfFacing,Label).
comp_preferredLabelOfFacing(Facing,Label) :-
  rdf_has(Facing,shop:labelOfFacing,Label).
comp_preferredLabelOfFacing(Facing,Label) :-
  % preferred if label is adjacent to facing without labelOfFacing
  owl_has(Facing,shop:adjacentLabelOfFacing,Label),
  \+ rdf_has(Facing,shop:labelOfFacing,_).

%% comp_isSpaceRemainingInFacing
%
comp_isSpaceRemainingInFacing(Facing,Val_XSD) :-
  shelf_facing_products(Facing, ProductsFrontToBack),
  ((ProductsFrontToBack=[] ; (
    reverse(ProductsFrontToBack, ProductsBackToFront),
    ProductsFrontToBack=[(_,First)|_],
    ProductsBackToFront=[(_,Last)|_], (
    facing_space_remaining_in_front(Facing,First);
    facing_space_remaining_behind(Facing,Last))
  )) ->
  xsd_boolean('true',Val_XSD);
  xsd_boolean('false',Val_XSD)),!.

%% comp_facingPose
%
comp_facingPose(Facing, Pose) :-
  rdf_has(Facing, shop:leftSeparator, _), !,
  rdf_has(Facing, shop:layerOfFacing, Layer),
  object_dimensions(Facing, _, _, Facing_H),
  shelf_facing_position(Facing, Pos_X),
  Pos_Y is -0.06,               % 0.06 to leave some room at the front and back of the facing
  Pos_Z is 0.5*Facing_H + 0.05, % 0.05 pushes ontop of supporting plane
  owl_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose',
    [pose=(Layer,[Pos_X,Pos_Y,Pos_Z],[0.0,0.0,0.0,1.0])], Pose).
comp_facingPose(Facing, Pose) :-
  rdf_has(Facing, shop:mountingBarOfFacing, _), !,
  rdf_has(Facing, shop:layerOfFacing, Layer),
  comp_facingHeight(Facing, literal(type(_,Facing_H_Atom))),
  atom_number(Facing_H_Atom, Facing_H),
  shelf_facing_position(Facing,Pos_X),
  Pos_Y is -0.03,           
  Pos_Z is -0.5*Facing_H, 
  owl_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose',
    [pose=(Layer,[Pos_X,Pos_Y,Pos_Z],[0.0,0.0,0.0,1.0])], Pose).
  
%% comp_facingWidth
%
comp_facingWidth(Facing, XSD_Val) :-
  shelf_facing_width(Facing,Value),
  xsd_float(Value, XSD_Val).

shelf_facing_width(Facing, Value) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  rdf_has(Facing, shop:leftSeparator, Left),
  rdf_has(Facing, shop:rightSeparator, Right),
  shelf_layer_position(ShelfLayer, Left, Pos_Left),
  shelf_layer_position(ShelfLayer, Right, Pos_Right),
  Value is abs(Pos_Right - Pos_Left). % leave 2cm to each side
shelf_facing_width(Facing, Value) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_mounting(ShelfLayer), !,
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar),
  shelf_layer_position(ShelfLayer, MountingBar, MountingBarPos),
  object_dimensions(ShelfLayer, _, LayerWidth, _),
  ( rdf_has(Facing, shop:leftMountingBar, Left) ->
    shelf_layer_position(ShelfLayer, Left, LeftPos) ;
    LeftPos is -0.5*LayerWidth
  ),
  ( rdf_has(Facing, shop:rightMountingBar, Right) ->
    shelf_layer_position(ShelfLayer, Right, RightPos) ;
    RightPos is 0.5*LayerWidth
  ),
  Value is min(MountingBarPos - LeftPos,
               RightPos - MountingBarPos). % leave 1cm to each side

%% comp_facingHeight
%
comp_facingHeight(Facing, XSD_Val) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  belief_at_relative_to(ShelfLayer, ShelfFrame, [_,_,[_,_,X_Pos],_]),
  % compute distance to layer above
  ( shelf_layer_above(ShelfLayer, LayerAbove) -> (
    belief_at_relative_to(LayerAbove, ShelfFrame, [_,_,[_,_,Y_Pos],_]),
    Distance is abs(X_Pos-Y_Pos)) ; (
    % no layer above
    object_dimensions(ShelfFrame, _, _, Frame_H),
    Distance is 0.5*Frame_H - X_Pos
  )),
  % compute available space for this facing
  ( shelf_layer_standing(LayerAbove) -> % FIXME could be unbound
    % above is also standing layer, whole space can be taken TODO minus layer height
    Value is Distance - 0.1;
    % above is mounting layer, space must be shared. HACK For now assume equal space sharing
    Value is 0.5*Distance - 0.1
  ),
  xsd_float(Value, XSD_Val).
comp_facingHeight(Facing, XSD_Val) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_mounting(ShelfLayer), !,
  shelf_layer_frame(ShelfLayer, ShelfFrame),
  belief_at_relative_to(ShelfLayer, ShelfFrame, [_,_,[_,_,X_Pos],_]),
  % compute distance to layer above
  ( shelf_layer_below(ShelfLayer, LayerBelow) -> (
    belief_at_relative_to(LayerBelow, ShelfFrame, [_,_,[_,_,Y_Pos],_]),
    Distance is abs(X_Pos-Y_Pos)) ; (
    % no layer below
    object_dimensions(ShelfFrame, _, _, Frame_H),
    Distance is 0.5*Frame_H + X_Pos
  )),
  % compute available space for this facing
  ( shelf_layer_mounting(LayerBelow) ->  % FIXME could be unbound
    % below is also mounting layer, whole space can be taken TODO minus layer height
    Value is Distance - 0.1;
    % below is standing layer, space must be shared. HACK For now assume equal space sharing
    Value is 0.5*Distance - 0.1
  ),
  xsd_float(Value, XSD_Val).

%% comp_facingDepth
%
comp_facingDepth(Facing, XSD_Val) :- comp_facingDepth(Facing, shelf_layer_standing, -0.06, XSD_Val).
comp_facingDepth(Facing, XSD_Val) :- comp_facingDepth(Facing, shelf_layer_mounting, 0.0, XSD_Val).
comp_facingDepth(Facing, Selector, Offset, XSD_Val) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  call(Selector, ShelfLayer), !,
  object_dimensions(ShelfLayer, Value, _, _),
  Value_ is Value + Offset,
  xsd_float(Value_, XSD_Val).

comp_mainColorOfFacing(Facing, Color_XSD) :-
  rdf_has(Facing, shop:layerOfFacing, _), !,
  ((owl_individual_of_during(Facing, shop:'UnlabeledProductFacing'),Col='1.0 0.35 0.0 0.4');
   % FIXME: MisplacedProductFacing seems slow, comp_MisplacedProductFacing is a bit faster
   (comp_MisplacedProductFacing(Facing),Col='1.0 0.0 0.0 0.4');
   %(owl_individual_of_during(Facing, shop:'MisplacedProductFacing'),Col='1.0 0.0 0.0 0.4');
   (owl_individual_of_during(Facing, shop:'EmptyProductFacing'),Col='1.0 1.0 0.0 0.4');
   (owl_individual_of_during(Facing, shop:'FullProductFacing'),Col='0.0 0.25 0.0 0.4');
   Col='0.0 1.0 0.0 0.4'),
  Color_XSD=literal(type(xsd:string, Col)), !.

comp_MisplacedProductFacing(Facing) :-
  rdf_has(Facing,shop:productInFacing,Product),
  owl_has(Product,shop:articleNumberOfProduct,AN),
  forall( rdf_has(Label,shop:articleNumberOfLabel,AN),
          \+ comp_preferredLabelOfFacing(Facing,Label) ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% belief_state part of shelves

%%
%
% This predicate exists to establish some relations
% between labels and facings, and to create facings
% between separators and mounting bars.
%
belief_shelf_part_at(Frame, Type, Pos, Obj) :-
  rdfs_subclass_of(Type, shop:'ShelfLayer'), !,
  pos_term(y,Pos,PosTerm),
  belief_perceived_part_at_axis(Frame, Type, PosTerm, Obj).

belief_shelf_part_at(Layer, Type, Pos, Obj) :-
  rdfs_subclass_of(Type, shop:'ShelfSeparator'), !,
  pos_term(x,Pos,PosTerm),
  belief_perceived_part_at_axis(Layer, Type, PosTerm, Obj),
  shelf_separator_insert(Layer,Obj).

belief_shelf_part_at(Layer, Type, Pos, Obj) :-
  rdfs_subclass_of(Type, shop:'ShelfMountingBar'), !,
  pos_term(x,Pos,PosTerm),
  belief_perceived_part_at_axis(Layer, Type, PosTerm, Obj),
  shelf_mounting_bar_insert(Layer,Obj).

belief_shelf_part_at(Layer, Type, Pos, Obj) :-
  rdfs_subclass_of(Type, shop:'ShelfLabel'), !,
  pos_term(x,Pos,PosTerm),
  belief_perceived_part_at_axis(Layer, Type, PosTerm, Obj),
  shelf_label_insert(Layer,Obj).

belief_shelf_barcode_at(Layer, Type, ArticleNumber_value, PosNorm, Obj) :-
  create_article_number(ArticleNumber_value, ArticleNumber),
  belief_shelf_part_at(Layer, Type, PosNorm, Obj),
  rdf_assert(Obj, shop:articleNumberOfLabel, ArticleNumber).

pos_term(Axis, norm(Pos), norm(Axis,Pos)) :- !.
pos_term(Axis, Pos, pos(Axis,Pos)).

product_dimensions(X,D,W,H):-object_dimensions(X,D,W,H), !.
product_dimensions(_,0.04,0.04,0.04).

product_spawn_at(Facing, Type, Offset_D, Obj) :-
  rdf_has(Facing, shop:layerOfFacing, Layer),
  
  product_type_dimensions(Type, [Obj_D,_,_]),
  object_dimensions(Layer,Layer_D,_,_),
  Layer_D*0.5 > Offset_D + Obj_D*0.5 + 0.04,
  
  belief_new_object(Type, Obj),
  % enforce we have a product here
  ( rdfs_individual_of(Obj,shop:'Product') -> true ;(
    print_message(warning, shop([Obj], 'Is not subclass of shop:Product.')),
    rdf_assert(Obj,rdf:type,shop:'Product') )),
  
  % compute offset
  product_dimensions(Obj,_,_,Obj_H),
  belief_at_id(Facing, [_,_,[Facing_X,_,_],_]),
  
  ( shelf_layer_standing(Layer) ->
    Offset_H is Obj_H*0.5 + 0.05 ;
    Offset_H is -Obj_H*0.5 - 0.05 ),
  
  % HACK rotate if it has a mesh
  ( object_mesh_path(Obj,_) ->
    Rot=[0.0, 0.0, -0.70711, 0.70711] ;
    Rot=[0.0, 0.0, 0.0, 1.0] ),
  writeln(Rot),
  
  % declare transform
  object_frame_name(Layer, Layer_frame),
  belief_at_update(Obj, [Layer_frame,_, 
      [Facing_X, Offset_D, Offset_H],
      Rot]),
  rdf_assert(Facing, shop:productInFacing, Obj, belief_state),
  
  belief_republish_objects([Facing]).

product_spawn_front_to_back(Facing, Obj) :-
  shelf_facing_product_type(Facing, ProductType),
  product_spawn_front_to_back(Facing, Obj, ProductType).
  
product_spawn_front_to_back(Facing, Obj, Type) :-
  rdf_has(Facing, shop:layerOfFacing, Layer),
  product_type_dimensions(Type, [Obj_D,_,_]),
  shelf_facing_products(Facing, ProductsFrontToBack),
  reverse(ProductsFrontToBack, ProductsBackToFront),
  ( ProductsBackToFront=[] -> (
    object_dimensions(Layer,Layer_D,_,_),
    Obj_Pos is -Layer_D*0.5 + Obj_D*0.5 + 0.01,
    product_spawn_at(Facing, Type, Obj_Pos, Obj));(
    ProductsBackToFront=[(Last_Pos,Last)|_],
    product_dimensions(Last,Last_D,_,_),
    Obj_Pos is Last_Pos + 0.5*Last_D + 0.5*Obj_D + 0.02,
    product_spawn_at(Facing, Type, Obj_Pos, Obj)
  )).
  
shelf_facing_products(Facing, Products) :-
  findall((Pos,Product), (
    rdf_has(Facing, shop:productInFacing, Product),
    belief_at_id(Product, [_,_,[_,Pos,_],_])), Products_unsorted),
  sort(Products_unsorted, Products).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Helper predicates

max_negative_element([(_,D_A)|Xs], (Needle,D_Needle)) :-
  D_A > 0.0, !, max_negative_element(Xs, (Needle,D_Needle)).
max_negative_element([(A,D_A)|Rest], (Needle,D_Needle)) :-
  max_negative_element(Rest, (B,D_B)),
  ( D_A > D_B -> (
    Needle=A, D_Needle=D_A );(
    Needle=B, D_Needle=D_B )).
max_negative_element([(A,D_A)|_], (A,D_A)).

min_positive_element([(_,D_A)|Xs], (Needle,D_Needle)) :-
  D_A < 0.0, !, min_positive_element(Xs, (Needle,D_Needle)).
min_positive_element([(A,D_A)|Rest], (Needle,D_Needle)) :-
  min_positive_element(Rest, (B,D_B)),
  ( D_A < D_B ->
    Needle=A, D_Needle=D_A;
    Needle=B, D_Needle=D_B ).
min_positive_element([(A,D_A)|_], (A,D_A)).
