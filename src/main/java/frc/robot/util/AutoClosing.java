package frc.robot.util;

import java.lang.reflect.Field;
import java.lang.reflect.Method;

public interface AutoClosing extends AutoCloseable {
  @Override
  default void close() throws Exception {
    Class<?> clazz = this.getClass();
    Field[] fields = clazz.getDeclaredFields();
    for (Field field : fields) {
      try {
        Class<?> fieldType = field.getType();
        try {
          Method runMethod = fieldType.getMethod("close");
          // Invoke the run method if it exists
          runMethod.invoke(field.get(this));
        } catch (NoSuchMethodException e) {

        }
      } catch (IllegalAccessException e) {
      }
    }
  }
}
