module.exports = (sequelize, DataTypes) =>
sequelize.define('users', {
  email: { type: DataTypes.STRING, primaryKey: true },
  password: { type: DataTypes.STRING }
}, {
  timestamps: false
});
