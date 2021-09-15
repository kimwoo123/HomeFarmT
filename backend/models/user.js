module.exports = (sequelize, DataTypes) =>
sequelize.define('users', {
  email: { type: DataTypes.STRING, primaryKey: true, validate: {
    isEmail: true
  }},
  password: { type: DataTypes.STRING, }
}, {
  timestamps: false
});
