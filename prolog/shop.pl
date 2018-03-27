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
%    - only load limited product catalog
%    - add conversation routine between ean/dan?
%    - compute same as between ProductWithDAN/ProductWithEAN?
%    - compute/assert same as between EAN/DAN ArticluNumber instances!
%    - use qudt and ensure unit is meters here.
%    - shop:productInFacing must be updated when facings manipulated!
%    - potentially there will be problems if label perception is drastically
%        messed up, when the shelf structure is asserted first time.
%        i.e., if this messes up ordering, because at the moment it is expected that ordering
%        never changes once perceived.
%        should add a check if asserted spatial relations of re-preceived parts still hold
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
      shelf_facing_total_space/2,
      shelf_facing_free_space/2,
      shelf_facing_occupied_space/2,
      % computable properties
      comp_isSpaceRemainingInFacing/2,
      comp_facingPose/2,
      comp_facingWidth/2,
      comp_facingHeight/2,
      comp_facingDepth/2,
      %%%%%
      shelf_find_parent/2,
      shelf_layer_spawn/4,
      shelf_layer_spawn_label/5,
      shelf_facing_spawn_front/2,
      shelf_facing_spawn_front/3,
      shelf_facing_spawn_back/2,
      shelf_facing_spawn_back/3
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
    shelf_layer_spawn(r,r,+,-),
    shelf_layer_spawn_label(r,r,+,+,-),
    shelf_facing(r,r),
    shelf_facing_product_type(r,r),
    shelf_facing_total_space(r,?),
    shelf_facing_free_space(r,-),
    shelf_facing_occupied_space(r,-),
    shelf_find_parent(r,r).

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

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% 

create_article_number(ean(EAN), ArticleNumber) :-
  owl_has(ArticleNumber, shop:articleNumberString, literal(type(shop:ean,EAN))), !.
create_article_number(ean(EAN), ArticleNumber) :-
  owl_instance_from_class(shop:'ArticleNumber',ArticleNumber),
  rdf_assert(ArticleNumber, shop:ean, literal(type(shop:ean, EAN))).

create_article_number(dan(DAN), ArticleNumber) :-
  owl_has(ArticleNumber, shop:articleNumberString, literal(type(shop:dan,DAN))), !.
create_article_number(dan(DAN), ArticleNumber) :-
  owl_instance_from_class(shop:'ArticleNumber',ArticleNumber),
  rdf_assert(ArticleNumber, shop:ean, literal(type(shop:dan, DAN))).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Shelves

%% 
shelf_find_parent(Object, Parent) :-
  rdfs_individual_of(Object, shop:'ShelfLayer'), !,
  shelf_find_frame_of_object(Object, Parent).
shelf_find_parent(Object, Parent) :-
  shelf_find_frame_of_object(Object, Frame),
  shelf_find_layer_of_object(Object, Frame, Parent).

shelf_find_frame_of_object(Obj, Frame) :-
  findall(X, rdfs_individual_of(X, shop:'ShelfFrame'), Xs),
  closest_object(Obj, Xs, Frame, _).
shelf_find_layer_of_object(Obj, Frame, Layer) :-
  rdfs_individual_of(Frame, shop:'ShelfFrame'),
  findall(X, (
    rdf_has(Frame, knowrob:properPhysicalParts, X),
    rdfs_individual_of(X, shop:'ShelfLayer')
  ), Xs),
  closest_object(Obj, Xs, Layer, _).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLayer'

%% 
shelf_layer_frame(Layer, Frame) :-
  owl_has(Frame, knowrob:properPhysicalParts, Layer),
  rdfs_individual_of(Frame, shop:'ShelfFrame'), !.

%% 
shelf_layer_mounting(ShelfLayer) :- rdfs_individual_of(ShelfLayer, shop:'ShelfLayerMounting').
%% 
shelf_layer_standing(ShelfLayer) :- rdfs_individual_of(ShelfLayer, shop:'ShelfLayerStanding').

%% shelf_layer_position
%
% The position of some object on a shelf layer.
% Position is simply the x-value of the object's pose in the shelf layer's frame.
%
shelf_layer_position(Layer, Object, Position) :-
  belief_at_relative_to(Object, Layer, [_,_,[Position,_,_],_]).

%% 
shelf_layer_above(ShelfLayer, AboveLayer) :-
  shelf_layer_sibling(ShelfLayer, max_negative_element, AboveLayer).
%% 
shelf_layer_below(ShelfLayer, BelowLayer) :-
  shelf_layer_sibling(ShelfLayer, min_positive_element, BelowLayer).

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
    D is NeedlePos - Pos_X
  ), Positions).

shelf_layer_update_labels(ShelfLayer) :-
  % step through all labels, find surrounding faces corresponding
  % to this label and associate them to the article number
  findall(FacingGroup, (
    shelf_layer_label(ShelfLayer,Label),
    rdf_has(LabeledFacing, shop:labelOfFacing, Label), (
    FacingGroup = [LabeledFacing] ; (
    shelf_labeled_facings(LabeledFacing, [LeftFacings,RightFacings]),
    shelf_facings_update_label(LeftFacings, Label),
    shelf_facings_update_label(RightFacings, Label),
    append(LeftFacings, [RightFacings], FacingGroup)
  ))), FacingGroups),
  flatten(FacingGroups, LabeledFacings),
  % retract article number for all remaining (orphan) facings
  forall((
    shelf_facing(ShelfLayer,Facing),
    \+ member(Facing,LabeledFacings)),
    rdf_retractall(Facing, shop:articleNumberOfFacing, _)
  ).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfSeparator'

%%
shelf_layer_separator(ShelfLayer, Separator) :-
  owl_has(ShelfLayer, knowrob:properPhysicalParts, Separator),
  rdfs_individual_of(Separator, shop:'ShelfSeparator').

%%
shelf_separator_insert(ShelfLayer,Separator) :-
  shelf_layer_standing(ShelfLayer),
  shelf_layer_neighbours(ShelfLayer, Separator, shelf_layer_separator, Xs),
  ( min_positive_element(Xs, (X,_)) -> 
    shelf_facing_assert(ShelfLayer,[Separator,X],_) ;
    true ),
  ( max_negative_element(Xs, (Y,_)) -> 
    shelf_facing_assert(ShelfLayer,[Y,Separator],_) ;
    true ),
  ( ground([X,Y]) -> (
    rdf_has(Facing, shop:leftSeparator, X),
    rdf_has(Facing, shop:rightSeparator, Y),
    shelf_facing_retract(Facing)) ; true ),
  shelf_layer_update_labels(ShelfLayer).
  

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ShelfMountingBar'

%%
shelf_layer_mounting_bar(ShelfLayer, MountingBar) :-
  owl_has(ShelfLayer, knowrob:properPhysicalParts, MountingBar),
  rdfs_individual_of(MountingBar, shop:'ShelfMountingBar').

shelf_mounting_bar_insert(ShelfLayer,MountingBar) :-
  shelf_layer_mounting(ShelfLayer),
  shelf_facing_assert(ShelfLayer,MountingBar,Facing),
  shelf_layer_neighbours(ShelfLayer, MountingBar, shelf_layer_mounting_bar, Xs),
  ( min_positive_element(Xs, (Left,_)) -> (
    rdf_assert(Facing, shop:leftMountingBar, Left, belief_state),
    rdf_has(LeftFacing, shop:mountingBarOfFacing, Left),
    rdf_retractall(LeftFacing, shop:rightMountingBar, _),
    rdf_assert(LeftFacing, shop:rightMountingBar, MountingBar, belief_state) ) ;
    true ),
  ( max_negative_element(Xs, (Right,_)) -> (
    rdf_assert(Facing, shop:rightMountingBar, Right, belief_state),
    rdf_has(RightFacing, shop:mountingBarOfFacing, Right),
    rdf_retractall(RightFacing, shop:leftMountingBar, _),
    rdf_assert(RightFacing, shop:leftMountingBar, MountingBar, belief_state) ) ;
    true ),
  % update the mounting_bar-label association
  shelf_layer_update_labels(ShelfLayer).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% knowrob:'ProductFacing'

%%
shelf_facing(ShelfLayer, Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer).

shelf_facing_assert(ShelfLayer,[Left,Right],Facing) :-
  shelf_layer_standing(ShelfLayer), !,
  rdfs_individual_of(Left, shop:'ShelfSeparator'),
  rdfs_individual_of(Right, shop:'ShelfSeparator'),
  rdf_instance_from_class(shop:'ProductFacingStanding', belief_state, Facing),
  rdf_assert(Facing, shop:leftSeparator, Left, belief_state),
  rdf_assert(Facing, shop:rightSeparator, Right, belief_state),
  rdf_assert(Facing, shop:layerOfFacing, ShelfLayer, belief_state),
  belief_republish_objects([Facing]).

shelf_facing_assert(ShelfLayer,MountingBar,Facing) :-
  shelf_layer_mounting(ShelfLayer), !,
  rdfs_individual_of(MountingBar, shop:'ShelfMountingBar'),
  rdf_instance_from_class(shop:'ProductFacingMounting', belief_state, Facing),
  rdf_assert(Facing, shop:mountingBarOfFacing, MountingBar, belief_state),
  rdf_assert(Facing, shop:layerOfFacing, ShelfLayer, belief_state),
  belief_republish_objects([Facing]).

shelf_facing_retract(Facing) :-
  rdf_retractall(Facing, _, _).

shelf_facings_between(F, F, []) :- !.
shelf_facings_between(Left, Right, Between) :-
  shelf_facing_next(Left, Next),
  ( Next = Right -> Between = [] ; (
    shelf_facings_between(Next, Right,Rest),
    Between = [Next|Rest]
  )).

shelf_facings_before(Facing, LeftToRight) :-
  shelf_facings_before_(Facing, RightToLeft),
  reverse(RightToLeft, LeftToRight).
shelf_facings_before_(Facing, [Left|Rest]) :-
  shelf_facing_previous(Facing, Left),
  shelf_facings_before_(Left, Rest), !.
shelf_facings_before_(_, []).
  
shelf_facings_after(Facing, [Right|Rest]) :-
  shelf_facing_next(Facing, Right),
  shelf_facings_after(Right, Rest), !.
shelf_facings_after(_, []).

%% TODO: use shop:leftProductFacing instead
shelf_facing_previous(Facing, Prev) :-
  rdf_has(Facing, shop:leftSeparator, X),
  rdf_has(Prev, shop:rightSeparator, X), !.
shelf_facing_previous(Facing, Prev) :-
  rdf_has(Facing, shop:leftMountingBar, X),
  rdf_has(Prev, shop:mountingBarOfFacing, X).

%% TODO: use shop:rightProductFacing instead
shelf_facing_next(Facing, Next) :-
  rdf_has(Facing, shop:rightSeparator, X),
  rdf_has(Next, shop:leftSeparator, X), !.
shelf_facing_next(Facing, Next) :-
  rdf_has(Facing, shop:rightMountingBar, X),
  rdf_has(Next, shop:mountingBarOfFacing, X).

%% shelf_facing_total_space
%
shelf_facing_total_space(Facing, Size) :-
  rdfs_individual_of(Facing, shop:'ProductFacing'),
  % TODO: not accurate! there is some space that can not be used by products!
  %       extend the owl model with this information
  owl_has(Facing, knowrob:depthOfObject, Depth), !,
  strip_literal_type(Depth, SizeAtom),
  atom_number(SizeAtom, Size).

%% shelf_facing_occupied_space
%
shelf_facing_occupied_space(Facing, OccupiedSpace) :-
  rdfs_individual_of(Facing, shop:'ProductFacing'),
  findall(Size, (
    rdf_has(Facing, shop:productInFacing, Product),
    object_dimensions(Product, _, _, Size)
  ), Sizes),
  sumlist(Sizes, OccupiedSpace).

%% shelf_facing_free_space
%
shelf_facing_free_space(Facing, Remaining) :-
  shelf_facing_total_space(Facing, Total),
  shelf_facing_occupied_space(Facing, Occupied),
  Remaining is Total - Occupied.

%% shelf_facing_product_type
%
shelf_facing_product_type(Facing, ProductType) :-
  once(owl_has(Facing, shop:articleNumberOfFacing, ArticleNumber)),
  rdf_has(R, owl:hasValue, ArticleNumber),
  rdf_has(R, owl:onProperty, shop:articleNumberOfProduct),
  rdf_has(ProductType, rdfs:subClassOf, R),
  rdf_has(ProductType, rdf:type, owl:'Class'), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% shop:'ShelfLabel'

shelf_layer_label(ShelfLayer, Label) :-
  rdfs_individual_of(ShelfLayer, shop:'ShelfLayer'),
  owl_has(ShelfLayer, knowrob:properPhysicalParts, Label),
  rdfs_individual_of(Label, shop:'ShelfLabel').

shelf_label_insert(ShelfLayer,ArticleNumber,Label) :-
  % assert the corresponding article number
  rdf_assert(Label, shop:articleNumberOfLabel, ArticleNumber),
  % find the position of Label on the shelf
  shelf_layer_position(ShelfLayer, Label, LabelPos),
  % first find the facing under which the label was perceived, 
  % then assert labelOfFacing and articleNumberOfFacing
  ( shelf_layer_find_facing_at(ShelfLayer,LabelPos,LabeledFacing) -> (
    rdf_retractall(LabeledFacing, shop:labelOfFacing, _), % FIXME: retract safe?
    rdf_retractall(LabeledFacing, shop:articleNumberOfFacing, _),
    rdf_assert(LabeledFacing, shop:labelOfFacing, Label, belief_state),
    rdf_assert(LabeledFacing, shop:articleNumberOfFacing, ArticleNumber, belief_state)
  ) ; true),
  % update the facing-label relation
  shelf_layer_update_labels(ShelfLayer).

shelf_layer_find_facing_at(ShelfLayer,Pos,Facing) :-
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  rdf_has(Facing, shop:leftSeparator, Left),
  rdf_has(Facing, shop:rightSeparator, Right),
  shelf_layer_position(ShelfLayer, Left, Left_Pos),
  shelf_layer_position(ShelfLayer, Right, Right_Pos),
  Left_Pos =< Pos, Right_Pos >= Pos, !.

shelf_labeled_facings(LabeledFacing, [LeftScope,RightScope]) :-
  % the scope of labels is influenced by how far away the next label 
  % is to the left and right. The facings in between are evenly distributed between
  % the adjacent labels.
  ( shelf_label_previous(LabeledFacing, LeftLabel) -> (
    rdf_has(PrevFacing, shop:labelOfFacing, LeftLabel),
    shelf_facings_between(PrevFacing,LabeledFacing,Facings),
    length(Facings, NumFacings), Count is round(NumFacings / 2),
    take_tail(Facings,NumFacings,LeftFacings )) ; (
    shelf_facings_before(LabeledFacing, LeftFacings)
  )),
  ( shelf_label_next(LabeledFacing, RightLabel) -> (
    rdf_has(NextFacing, shop:labelOfFacing, RightLabel),
    shelf_facings_between(LabeledFacing,NextFacing,Facings),
    length(Facings, NumFacings), Count is round(NumFacings / 2),
    take_head(Facings,NumFacings,RightFacings )) ; (
    shelf_facings_after(LabeledFacing, RightFacings)
  )),
  % number of facings to the left and right which are understood to be labeled 
  % by Label must be evenly distributed, and identical in number to the left and
  % right of the label.
  length(LeftFacings, Left_count),
  length(RightFacings, Right_count),
  Count is min(Left_count,Right_count),
  take_tail(LeftFacings,Count,LeftScope),
  take_head(RightFacings,Count,RightScope).

shelf_facing_update_label(Facing, Label) :-
  rdf_has(Label, shop:articleNumberOfLabel, ArticleNumber),
  rdf_retractall(Facing, shop:articleNumberOfFacing, _),
  rdf_assert(Facing, shop:articleNumberOfFacing, ArticleNumber).

shelf_facings_update_label([], _) :- !.
shelf_facings_update_label([F|Rest], Label) :-
  shelf_facing_update_label(F, Label),
  shelf_facings_update_label(Rest, Label).

%%
shelf_label_previous(Facing, LeftLabel) :-
  shelf_facing_previous(Facing, LeftFacing),
  ( rdf_has(LeftFacing, shop:labelOfFacing, LeftLabel) ;
    shelf_label_previous(LeftFacing, LeftLabel) ), !.
%%
shelf_label_next(Facing, RightLabel) :-
  shelf_facing_next(Facing, RightFacing),
  ( rdf_has(RightFacing, shop:labelOfFacing, RightLabel) ;
    shelf_label_next(RightFacing, RightLabel) ), !.

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% COMPUTABLE shop:'isSpaceRemainingInFacing'

%% comp_isSpaceRemainingInFacing
%
%comp_isSpaceRemainingInFacing(Facing,
%  literal(type('http://www.w3.org/2001/XMLSchema#boolean', true))) :-
%  shelf_facing_free_space(Facing, Remaining),
%  ( shelf_facing_product_type(Facing, ProductType) -> (
%    object_class_dimensions(ProductType, _, _, ProductSize),
%    Remaining > ProductSize );
%    true % assume space is remaining if the facing does not have an associated product type
%  ), !.
comp_isSpaceRemainingInFacing(_,
  literal(type('http://www.w3.org/2001/XMLSchema#boolean', true))).

%% comp_facingPose
%
comp_facingPose(Facing, Pose) :-
  rdf_has(Facing, shop:mountingBarOfFacing, MountingBar), !,
  rdf_has(Facing, shop:layerOfFacing, Layer),
  comp_facingHeight(Facing, literal(type(_,Facing_H_Atom))),
  atom_number(Facing_H_Atom, Facing_H),
  belief_at_relative_to(MountingBar,  Layer, [_,_,[Pos_MountingBar,_,_],_]),
  Pos_X is -Pos_MountingBar,
  Pos_Y is -0.03,           
  Pos_Z is -0.5*Facing_H, 
  owl_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose',
    [pose=(Layer,[Pos_X,Pos_Y,Pos_Z],[0.0,0.0,0.0,1.0])], Pose).
comp_facingPose(Facing, Pose) :-
  rdf_has(Facing, shop:leftSeparator, Left), !,
  rdf_has(Facing, shop:rightSeparator, Right),
  rdf_has(Facing, shop:layerOfFacing, Layer),
  object_dimensions(Facing, _, _, Facing_H),
  belief_at_relative_to(Left,  Layer, [_,_,[Pos_Left,_,_],_]),
  belief_at_relative_to(Right, Layer, [_,_,[Pos_Right,_,_],_]),
  Pos_X is -0.5*(Pos_Left+Pos_Right),
  Pos_Y is -0.06,               % 0.06 to leave some room at the front and back of the facing
  Pos_Z is 0.5*Facing_H + 0.05, % 0.05 pushes ontop of supporting plane
  owl_instance_from_class('http://knowrob.org/kb/knowrob.owl#Pose',
    [pose=(Layer,[Pos_X,Pos_Y,Pos_Z],[0.0,0.0,0.0,1.0])], Pose).

%% comp_facingWidth
%
comp_facingWidth(Facing, XSD_Val) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  shelf_layer_standing(ShelfLayer), !,
  rdf_has(Facing, shop:leftSeparator, Left),
  rdf_has(Facing, shop:rightSeparator, Right),
  shelf_layer_position(ShelfLayer, Left, Pos_Left),
  shelf_layer_position(ShelfLayer, Right, Pos_Right),
  Value is abs(Pos_Right - Pos_Left)-0.04,
  xsd_float(Value, XSD_Val).
comp_facingWidth(Facing, XSD_Val) :-
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
               RightPos - MountingBarPos)-0.02,
  xsd_float(Value, XSD_Val).

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
comp_facingDepth(Facing, XSD_Val) :-
  comp_facingDepth(Facing, shelf_layer_standing, -0.06, XSD_Val).
comp_facingDepth(Facing, XSD_Val) :-
  comp_facingDepth(Facing, shelf_layer_mounting, 0.0, XSD_Val).
comp_facingDepth(Facing, Selector, Offset, XSD_Val) :-
  atom(Facing),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  call(Selector, ShelfLayer), !,
  object_dimensions(ShelfLayer, Value, _, _),
  Value_ is Value + Offset,
  xsd_float(Value_, XSD_Val).

comp_mainColorOfFacing(Facing, Color_XSD) :-
  rdf_has(Facing, shop:layerOfFacing, _), !,
  % TODO is there really no classification for #articleNumberOfFacing = 0 ??
  (
   % (RED)    productInFacing some MisplacedProduct
   (owl_individual_of(Facing, shop:'MisplacedProductFacing'),Col='1.0 0.0 0.0 0.5');
   % (YELLOW) #articleNumberOfFacing = 1 & #productInFacing=0
   (owl_individual_of(Facing, shop:'EmptyProductFacing'),Col='1.0 1.0 0.0 0.5');
   % (ORANGE) #articleNumberOfFacing = 0
   % TODO Facing class
   (\+ rdf_has(Facing, shop:articleNumberOfFacing, _),Col='1.0 0.35 0.0 0.5');
   % #articleNumberOfFacing = 1 & isSpaceRemainingInFacing=false
   (owl_individual_of(Facing, shop:'FullProductFacing'),Col='1.0 1.0 1.0 0.5');
   % #articleNumberOfFacing = 1 & isSpaceRemainingInFacing=true
   (owl_individual_of(Facing, shop:'FreeProductFacing'),Col='0.0 0.0 1.0 0.5');
   Col='0.0 1.0 0.0 0.5'),
  Color_XSD=literal(type(xsd:string, Col)), !.


% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Spawning new objects

belief_part_type_at(Parent, PartType, Part, Pos, Rot) :-
  belief_new_object(PartType, Part), % TODO is it a new object?
  object_frame_name(Parent, Parent_frame),
  belief_at_update(Part, [Parent_frame,_, Pos, Rot]),
  rdf_assert(Parent, knowrob:properPhysicalParts, Part, belief_state).

shelf_layer_spawn_pos(Layer, NormalizedPosition, SpawnPos) :-
  object_dimensions(Layer, _, Width, _),
  ( shelf_layer_standing(Layer) ->
    Width_reduced is Width - 0.02 ;
    Width_reduced is Width - 0.08 ),
  SpawnPos is Width_reduced*NormalizedPosition - 0.5*Width_reduced.

shelf_layer_spawn(Layer, Type, NormalizedPosition, Obj) :-
  rdfs_subclass_of(Type, shop:'ShelfSeparator'), !,
  shelf_layer_standing(Layer),
  shelf_layer_spawn_pos(Layer, NormalizedPosition, SpawnPos),
  belief_part_type_at(Layer, Type, Obj,
      [SpawnPos, -0.05, 0.07 ],
      [0.0, 0.0, 0.0, 1.0]),
  shelf_separator_insert(Layer,Obj).

shelf_layer_spawn(Layer, Type, NormalizedPosition, Obj) :-
  rdfs_subclass_of(Type, shop:'ShelfMountingBar'), !,
  shelf_layer_mounting(Layer),
  shelf_layer_spawn_pos(Layer, NormalizedPosition, SpawnPos),
  belief_part_type_at(Layer, Type, Obj,
      [SpawnPos, -0.02, -0.02 ],
      [0.0, 0.0, 0.0, 1.0]),
  shelf_mounting_bar_insert(Layer,Obj).

shelf_layer_spawn_label(Layer, Type, ArticleNumber_value, NormalizedPosition, Obj) :-
  rdfs_subclass_of(Type, shop:'ShelfLabel'), !,
  shelf_layer_standing(Layer),
  shelf_layer_spawn_pos(Layer, NormalizedPosition, SpawnPos),
  belief_part_type_at(Layer, Type, Obj,
      [SpawnPos, -0.265, 0.04 ],
      [0.0, 0.0, 0.0, 1.0]),
  create_article_number(ArticleNumber_value, ArticleNumber),
  shelf_label_insert(Layer,ArticleNumber,Obj).

shelf_facing_spawn_front(Facing, Obj) :-
  rdfs_individual_of(Facing, shop:'ProductFacing'),
  shelf_facing_product_type(Facing, ProductType),
  shelf_facing_spawn_front(Facing, Obj, ProductType).

shelf_facing_spawn_front(Facing, Obj, ProductType) :-
  shelf_facing_spawn_back(Facing, Obj, ProductType). % TODO

shelf_facing_spawn_back(Facing, Obj) :-
  rdfs_individual_of(Facing, shop:'ProductFacing'),
  shelf_facing_product_type(Facing, ProductType),
  shelf_facing_spawn_back(Facing, Obj, ProductType).

shelf_facing_spawn_back(Facing, Obj, ProductType) :-
  belief_new_object(ProductType, Obj),
  rdf_has(Facing, shop:layerOfFacing, ShelfLayer),
  object_frame_name(ShelfLayer, ShelfLayer_frame),
  
  rdf_has(Facing, shop:leftSeparator, Left),
  rdf_has(Facing, shop:rightSeparator, Right),
  shelf_layer_position(ShelfLayer, Left, Pos_Left),
  shelf_layer_position(ShelfLayer, Right, Pos_Right),
  SpawnPos is -0.5*(Pos_Left+Pos_Right),
  
  belief_at_update(Obj, [ShelfLayer_frame,_, 
      [SpawnPos, 0.0, 0.1],
      [0.707107, 0.0, 0.0, 0.707106]]),
  rdf_assert(Facing, shop:productInFacing, Obj, belief_state).

% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %
% Helper predicates

take_head(_, 0, []) :- !.
take_head([X|Xs], N, [X|Ys]) :-
  M is N-1, take_head(Xs, M, Ys). 
take_tail(L, N, Tail) :-
  reverse(L,X), take_head(X,N,Y), reverse(Y,Tail).

closest_object(Obj, [X], X, D) :-
  object_distance(Obj, X, D), !.
closest_object(Obj, [X|Xs], Nearest, D) :-
  object_distance(Obj, X, D_X),
  closest_object(Obj, Xs, Nearest_Xs, D_Xs),
  ( D_Xs<D_X ->
  ( D=D_Xs, Nearest=Nearest_Xs);
  ( D=D_X,  Nearest=X)).

min_positive_element([(_,D_A)|Xs], (Needle,D_Needle)) :-
  D_A > 0.0, !, min_positive_element(Xs, (Needle,D_Needle)).
min_positive_element([(A,D_A)|Rest], (Needle,D_Needle)) :-
  min_positive_element(Rest, (B,D_B)),
  ( D_A > D_B -> (
    Needle=A, D_Needle=D_A );(
    Needle=B, D_Needle=D_B )).
min_positive_element([(A,D_A)|_], (A,D_A)).

max_negative_element([(_,D_A)|Xs], (Needle,D_Needle)) :-
  D_A < 0.0, !, max_negative_element(Xs, (Needle,D_Needle)).
max_negative_element([(A,D_A)|Rest], (Needle,D_Needle)) :-
  max_negative_element(Rest, (B,D_B)),
  ( D_A < D_B -> (
    Needle=A, D_Needle=D_A );(
    Needle=B, D_Needle=D_B )).
max_negative_element([(A,D_A)|_], (A,D_A)).
