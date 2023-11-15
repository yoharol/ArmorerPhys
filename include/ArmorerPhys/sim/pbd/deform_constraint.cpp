#include "ArmorerPhys/sim/pbd/deform_constraint.h"
#include "ArmorerPhys/type.h"

#include <Eigen/Dense>

namespace aphys {

template <int dim>
DeformConstraint<dim>::DeformConstraint(MatxXf& verts, MatxXf& verts_ref,
                                        const Matx3i& faces, Vecxf& face_mass,
                                        Vecxf& verts_invm, float hydro_alpha,
                                        float devia_alpha, float dt) {
  hydro_lambda.resize(faces.rows());
  devia_lambda.resize(faces.rows());
  hydro_tilde_alpha = hydro_alpha / (dt * dt);
  devia_tilde_alpha = devia_alpha / (dt * dt);
  project_func = [&]() {
    project(verts, verts_ref, faces, face_mass, verts_invm, dt);
  };
}

template <int dim>
void DeformConstraint<dim>::preProject() {
  for (int i = 0; i < hydro_lambda.size(); i++) {
    hydro_lambda(i) = 0.f;
    devia_lambda(i) = 0.f;
  }
}

template <>
void DeformConstraint<2>::project(MatxXf& verts, MatxXf& verts_ref,
                                  const Matx3i& faces, Vecxf& face_mass,
                                  Vecxf& verts_invm, float dt) {
  for (int i = 0; i < faces.rows(); i++) {
    int i1 = faces(i, 0);
    int i2 = faces(i, 1);
    int i3 = faces(i, 2);
    Vecxf p1 = verts.row(i1);
    Vecxf p2 = verts.row(i2);
    Vecxf p3 = verts.row(i3);
    Vecxf p1_ref = verts_ref.row(i1);
    Vecxf p2_ref = verts_ref.row(i2);
    Vecxf p3_ref = verts_ref.row(i3);
    float w1 = verts_invm(i1);
    float w2 = verts_invm(i2);
    float w3 = verts_invm(i3);
    MatxXf D(2, 2);
    D.col(0) = p1 - p3;
    D.col(1) = p2 - p3;
    MatxXf B(2, 2);
    B.col(0) = p1_ref - p3_ref;
    B.col(1) = p2_ref - p3_ref;
    B = B.inverse();
    MatxXf F = D * B;

    MatxXf par(6, 2);

    float C_H = F.determinant() - 1.f;
    MatxXf par_det_F(2, 2);
    par_det_F << F(1, 1), -F(1, 0), -F(0, 1), F(0, 0);
    MatxXf CH_H = par_det_F * (B.transpose());
    par.block(0, 0, 2, 1) = CH_H.col(0);
    par.block(2, 0, 2, 1) = CH_H.col(1);
    par.block(4, 0, 2, 1) = -(CH_H.col(0) + CH_H.col(1));

    float C_D = F.squaredNorm() - 2.0f;
    MatxXf CD_H = 2.0f * F * (B.transpose());
    par.block(0, 1, 2, 1) = CD_H.col(0);
    par.block(2, 1, 2, 1) = CD_H.col(1);
    par.block(4, 1, 2, 1) = -(CD_H.col(0) + CD_H.col(1));

    Eigen::DiagonalMatrix<float, 6> M(w1, w1, w2, w2, w3, w3);

    Mat2f C12 = par.transpose() * M * par;
    C12(0, 0) += hydro_tilde_alpha / face_mass(i);
    C12(1, 1) += devia_tilde_alpha / face_mass(i);
    Vec2f delta_lambda =
        -C12.inverse() *
        Vec2f(C_H + hydro_tilde_alpha * hydro_lambda(i) / face_mass(i),
              C_D + devia_tilde_alpha * devia_lambda(i) / face_mass(i));
    verts.row(i1) += w1 * delta_lambda(0) * par.block(0, 0, 2, 1).transpose() +
                     w1 * delta_lambda(1) * par.block(0, 1, 2, 1).transpose();
    verts.row(i2) += w2 * delta_lambda(0) * par.block(2, 0, 2, 1).transpose() +
                     w2 * delta_lambda(1) * par.block(2, 1, 2, 1).transpose();
    verts.row(i3) += w3 * delta_lambda(0) * par.block(4, 0, 2, 1).transpose() +
                     w3 * delta_lambda(1) * par.block(4, 1, 2, 1).transpose();
  }
}

template <>
void DeformConstraint<3>::project(MatxXf& verts, MatxXf& verts_ref,
                                  const Matx3i& faces, Vecxf& face_mass,
                                  Vecxf& verts_invm, float dt) {}

template struct DeformConstraint<2>;
template struct DeformConstraint<3>;

}  // namespace aphys